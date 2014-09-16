#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/message_filter.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>

// Generated messages
#include <dogsim/PathViewInfo.h>
#include <dogsim/PointArmCameraAction.h>

namespace {
using namespace dogsim;
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<dogsim::PointArmCameraAction> PointArmClient;
typedef vector<geometry_msgs::PointStamped> PointStampedVector;

static const ros::Duration FOCUS_TIMEOUT(1.0);
static const double PATH_VIS_THRESHOLD_DEFAULT = 0.99;
static const double LOOK_AT_PATH_WEIGHT_DEFAULT = 2;
static const double SEARCH_FOR_DOG_WEIGHT_DEFAULT = 1;
static const string STATE_NAMES[] = { "Idle", "Looking_At_Path", "Looking_for_Dog" };
static const string SEARCH_STATE_NAMES[] = { "None", "Last Known", "Head", "Arm", "Done" };
static const double SEARCH_CELL_SIZE_DEFAULT = 0.25;
static const double BASE_RADIUS = 0.668 / 2.0;
static const double CLUSTER_TOLERANCE_DEFAULT = 0.2;

static Eigen::Vector3f operator-(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs) {
    return Eigen::Vector3f(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

struct DistanceFromPoint {
    const geometry_msgs::PointStamped& curr;
    DistanceFromPoint(const geometry_msgs::PointStamped &p) :
        curr(p) {
    }
    double operator()(const geometry_msgs::PointStamped& a, const geometry_msgs::PointStamped& b) {
        return utils::pointToPointXYDistance(curr.point, a.point)
        < utils::pointToPointXYDistance(curr.point, b.point);
    }
};

struct HasPointInCell {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree;
    const double radiusSquared;

    HasPointInCell(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr aCloud,
            const pcl::KdTreeFLANN<pcl::PointXYZ>& aKdtree, const double aRadiusSquared) :
                cloud(aCloud), kdtree(aKdtree), radiusSquared(aRadiusSquared) {
    }

    bool operator()(const pcl::PointXYZ& p) {
        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) == 0) {
            ROS_DEBUG("No neighboring point in cloud");
            return false;
        }

        return pointNKNSquaredDistance[0] < radiusSquared;
    }
};

struct ClusterSmaller {
    bool operator()(const pcl::PointIndices& left, const pcl::PointIndices& right) {
        return left.indices.size() < right.indices.size();
    }
};

class FocusHead {
private:
    enum class ActionState {
        IDLE, LOOKING_AT_PATH, LOOKING_FOR_DOG
    };

    enum class SearchState {
        NONE, LAST_KNOWN, HEAD, ARM
    };

public:
    FocusHead() :
        pnh("~"), pointHeadClient("/head_traj_controller/point_head_action", true), pointArmClient(
                "point_arm_camera_action", true), state(ActionState::IDLE), searchState(
                        SearchState::NONE), currentActionScore(0), currentSearchOriginalSize(0), interrupts(
                                0), searchCloud(new pcl::PointCloud<pcl::PointXYZ>()) {

        nh.param("leash_length", leashLength, 2.0);
        nh.param("path_visibility_threshold", pathVisibilityThreshold, PATH_VIS_THRESHOLD_DEFAULT);
        pnh.param("look_at_path_weight", lookAtPathWeight, LOOK_AT_PATH_WEIGHT_DEFAULT);
        pnh.param("search_for_dog_weight", searchForDogWeight, SEARCH_FOR_DOG_WEIGHT_DEFAULT);
        pnh.param("search_cell_size", searchCellSize, SEARCH_CELL_SIZE_DEFAULT);
        pnh.param("cluster_tolerance", clusterTolerance, CLUSTER_TOLERANCE_DEFAULT);

        dogPositionSub.reset(
                new message_filters::Subscriber<DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogPositionSub->registerCallback(boost::bind(&FocusHead::dogPositionCallback, this, _1));

        pathViewSub.reset(
                new message_filters::Subscriber<PathViewInfo>(nh, "/path_visibility_detector/view",
                        1));
        pathViewSub->registerCallback(boost::bind(&FocusHead::pathViewCallback, this, _1));

        headSearchPointsSub.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,
                        "head_search_cloud_in", 1));
        headSearchPointsSub->registerCallback(boost::bind(&FocusHead::searchedPointsCB, this, _1));
        headSearchPointsSub->unsubscribe();

        armSearchPointsSub.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "arm_search_cloud_in",
                        1));
        armSearchPointsSub->registerCallback(boost::bind(&FocusHead::searchedPointsCB, this, _1));
        armSearchPointsSub->unsubscribe();

        pointHeadClient.waitForServer();
        pointArmClient.waitForServer();
        lookDirectionPub = nh.advertise<visualization_msgs::Marker>(
                "/focus_head_action/look_direction_viz", 1);

        searchCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/focus_head_action/search_cloud",
                1);
    }

private:

    void timeoutCallback(const ros::TimerEvent& e) {
        ROS_INFO("Point head action timed out");

        // Ignore any timeouts after completion.
        if (state == ActionState::IDLE) {
            ROS_DEBUG("Timeout received when state is idle");
            return;
        }

        if (state == ActionState::LOOKING_FOR_DOG) {
            ROS_DEBUG("Continuing search after timeout");
            executeNext(PointStamped(), ActionState::LOOKING_FOR_DOG, false);
            return;
        }

        assert(state == ActionState::LOOKING_AT_PATH);
        pointHeadClient.cancelGoal();
        pointArmClient.cancelGoal();
        resetState();
    }

    bool tryGoal(const ActionState actionType, const double score, const PointStamped position,
            bool isPositionSet) {
        ROS_DEBUG("Received a new goal of type %s with score %f and current score is %f",
                STATE_NAMES[static_cast<int>(actionType)].c_str(), score, currentActionScore);

        // Don't want to preempt because we could end up in a state where we never
        // finish looking at the path
        if (state == actionType) {
            ROS_DEBUG("Ignoring additional request for current action");
            return false;
        }

        if (score < currentActionScore) {
            ROS_DEBUG("Score for new action does not exceed current action");
            return false;
        }

        ROS_DEBUG("Accepted new goal with score %f (previous score was %f) and type %s", score,
                currentActionScore, STATE_NAMES[static_cast<int>(actionType)].c_str());
        currentActionScore = score;

        if (state != ActionState::IDLE) {
            ROS_DEBUG("Preempting action for new goal. Total interrupts: %u", interrupts);
            // Do not reset search state as we want the search to continue when it is
            // resumed.
            interrupts++;
            pointHeadClient.cancelGoal();
            pointArmClient.cancelGoal();

            if (state == ActionState::LOOKING_FOR_DOG) {
                unsubscribeToPoints();
            }
        }

        state = actionType;

        if (state == ActionState::LOOKING_FOR_DOG) {
            subscribeToPoints();
        }

        executeNext(position, actionType, isPositionSet);
        return true;
    }

    void subscribeToPoints() {
        ROS_DEBUG("Subscribing to points messages");
        headSearchPointsSub->subscribe();
        armSearchPointsSub->subscribe();
    }

    void unsubscribeToPoints() {
        ROS_DEBUG("Unsubscribing to points messages");
        headSearchPointsSub->unsubscribe();
        armSearchPointsSub->unsubscribe();
    }

    void resetSearch() {
        ROS_DEBUG("Resetting search state");
        searchState = SearchState::NONE;
        searchCloud->points.clear();
        publishSearchCloud();
    }

    void resetState() {
        state = ActionState::IDLE;
        currentActionScore = 0;
    }

    void resetSearchMap() {
        ROS_DEBUG("Initializing search map");

        // Generate a new search cloud.
        searchCloud->points.clear();
        searchedPoints.clear();

        const Point handPosition = handInBaseFrame().point;

        double sqPlanarLeashLength = utils::square(leashLength) - utils::square(handPosition.z);
        double planarLeashLength = sqrt(sqPlanarLeashLength);
        const pcl::PointXYZ planarHand(handPosition.x, handPosition.y, 0);

        // Assume the cells are circular and slightly overlap.
        double cellWidth = searchCellSize / sqrt(2.0);

        ROS_DEBUG("Creating a new search cloud for planar leash length %f and cell width %f",
                planarLeashLength, cellWidth);
        unsigned int size = currentSearchOriginalSize = static_cast<unsigned int>(ceil(
                planarLeashLength * 2 / cellWidth));

        // Always use an even size to simplify the math below.
        if (size % 2 == 1) {
            size++;
        }

        // The initial point is the bottom left corner.
        double x = handPosition.x - size / 2 * cellWidth + cellWidth / 2.0;
        for (unsigned int i = 0; i < size; ++i) {
            double y = handPosition.y - size / 2.0 * cellWidth + cellWidth / 2.0;
            for (unsigned int j = 0; j < size; ++j) {
                pcl::PointXYZ p(x, y, 0);
                // TODO: These checks do not account for partially covered cells.
                // Check if the point is inside the base
                if (fabs(x) < BASE_RADIUS && fabs(y) < BASE_RADIUS) {
                    ROS_DEBUG("Skipping point inside base @ %f %f", x, y);
                }
                // Check if the point is outside the radius of the leash
                else if (pcl::geometry::squaredDistance(p, planarHand) > sqPlanarLeashLength) {
                    ROS_DEBUG("Skipping point outside circle");
                }
                else {
                    searchCloud->points.push_back(p);
                }
                y += cellWidth;
            }
            x += cellWidth;
        }
        ROS_DEBUG("New search map has %lu points", searchCloud->points.size());
        publishSearchCloud();
    }

    PointStamped handInBaseFrame() const {
        // Use the current right hand position at height 0 as the starting point.
        // Determine the position of the hand in the base frame.
        PointStamped handInBaseFrame;
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "r_wrist_roll_link";
        while (!tf.waitForTransform("/base_footprint", handInHandFrame.header.frame_id,
                ros::Time(0), ros::Duration(30.0))) {
            ROS_DEBUG("Waiting for transform from /r_wrist_roll_link to /base_footprint");
        }
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame,
                    handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("Failed to transform hand position to /base_footprint");
            throw ex;
        }
        handInBaseFrame.header.frame_id = "/base_footprint";
        handInBaseFrame.header.stamp = ros::Time::now();
        return handInBaseFrame;
    }

    PointStamped cameraInFrame(const string& requestedFrame) const {
        PointStamped cameraInRequestedFrame;
        PointStamped cameraInCameraFrame;
        cameraInCameraFrame.header.frame_id = "wide_stereo_link";
        tf.waitForTransform(requestedFrame, cameraInCameraFrame.header.frame_id, ros::Time(0),
                ros::Duration(30.0));

        try {
            tf.transformPoint(requestedFrame, ros::Time(0), cameraInCameraFrame,
                    cameraInCameraFrame.header.frame_id, cameraInRequestedFrame);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("Failed to transform camera position to %s", requestedFrame.c_str());
            throw ex;
        }
        cameraInRequestedFrame.header.frame_id = requestedFrame;
        cameraInRequestedFrame.header.stamp = ros::Time::now();
        return cameraInRequestedFrame;
    }

    void pointArmCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const dogsim::PointArmCameraResultConstPtr result) {
        ROS_DEBUG("Received point arm complete callback");
        if (searchState != SearchState::ARM) {
            ROS_DEBUG("Spurious point arm complete callback received");
            return;
        }
        searchCompleteCallback();
    }

    void pointHeadCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const pr2_controllers_msgs::PointHeadResultConstPtr result) {
        ROS_DEBUG("Received point head complete callback");
        if (searchState != SearchState::HEAD) {
            ROS_DEBUG("Spurious point head complete callback received");
            return;
        }

        searchCompleteCallback();
    }

    void searchCompleteCallback() {

        if (state == ActionState::IDLE) {
            // Spurious late callback.
            return;
        }

        timeout.stop();
        if (state == ActionState::LOOKING_FOR_DOG) {
            executeNext(PointStamped(), ActionState::LOOKING_FOR_DOG, false);
        }
        // Looking at path currently.
        else {
            assert(state == ActionState::LOOKING_AT_PATH);
            resetState();
        }
    }

    void executeNext(const PointStamped& target, const ActionState targetType,
            const bool isPositionSet) {
        ROS_DEBUG("Executing search for target type %s with position set %u and target frame %s",
                STATE_NAMES[static_cast<int>(targetType)].c_str(), isPositionSet,
                target.header.frame_id.c_str());

        assert(!isPositionSet || target.header.frame_id.size() > 0);

        geometry_msgs::PointStamped finalTarget;
        if (targetType == ActionState::LOOKING_FOR_DOG) {
            // Determine if the search failed and should be reset
            if (searchCloud->points.size() == 0) {
                ROS_DEBUG(
                        "Resetting search due to either failed search or previous successful search.");
                resetSearch();
            }

            bool found = false;
            do {
                if (searchState == SearchState::NONE) {
                    bool proceeding = moveToNextSearchState();
                    assert(proceeding);
                }

                // If the search state is last known and there is no last known,
                // immediate increment the search.
                if (searchState == SearchState::LAST_KNOWN && !isPositionSet) {
                    ROS_DEBUG("Skipping last known search state because position is not set");
                    bool proceeding = moveToNextSearchState();
                    assert(proceeding);
                }

                ROS_DEBUG("Executing search step %s",
                        SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
                if (searchState == SearchState::LAST_KNOWN) {
                    ROS_DEBUG("Searching last known position");
                    finalTarget = target;
                    found = true;
                }
                else {
                    found = nextBestSearch(finalTarget);
                    if (found) {
                        searchedPoints.push_back(finalTarget);
                    }
                    else if(!moveToNextSearchState()){
                        ROS_WARN("Temporarily aborting search");
                        return;
                    }
                    ROS_DEBUG("Search target - x: %f, y: %f, z: %f", finalTarget.point.x,
                            finalTarget.point.y, finalTarget.point.z);
                }
            } while (!found);
        }
        else if (targetType == ActionState::LOOKING_AT_PATH) {
            ROS_DEBUG("Focusing head on path target in frame %s, %f %f %f",
                    target.header.frame_id.c_str(), target.point.x, target.point.y, target.point.z);
            assert(isPositionSet && "is position not set for path action");
            finalTarget = target;
        }
        else {
            assert(false && "Unknown target type");
        }

        ROS_DEBUG("Sending goal of type %s for %s",
                STATE_NAMES[static_cast<int>(targetType)].c_str(),
                SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());

        visualizeGoal(finalTarget, targetType);
        if (searchState == SearchState::ARM && targetType == ActionState::LOOKING_FOR_DOG) {
            dogsim::PointArmCameraGoal pacGoal;
            pacGoal.target = finalTarget;
            pointArmClient.sendGoal(pacGoal,
                    boost::bind(&FocusHead::pointArmCompleteCallback, this, _1, _2));
        }
        else {
            pr2_controllers_msgs::PointHeadGoal phGoal;
            phGoal.target = finalTarget;
            phGoal.pointing_frame = "wide_stereo_link";
            phGoal.pointing_axis.x = 1;
            phGoal.pointing_axis.y = 0;
            phGoal.pointing_axis.z = 0;

            // Pointing axis defaults to the x-axis.
            pointHeadClient.sendGoal(phGoal,
                    boost::bind(&FocusHead::pointHeadCompleteCallback, this, _1, _2));
        }
        timeout = nh.createTimer(FOCUS_TIMEOUT, &FocusHead::timeoutCallback, this,
                true /* One shot */);
        ROS_DEBUG("Focus target step completed");
    }

    void visualizeGoal(const PointStamped& goal, const ActionState targetType) const {
        static const std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
        static const std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);

        if (lookDirectionPub.getNumSubscribers() > 0) {
            visualization_msgs::Marker marker;
            marker.header = goal.header;
            marker.ns = "dogsim";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.points.resize(2);
            marker.points[0] = cameraInFrame(goal.header.frame_id).point;
            marker.points[1] = goal.point;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = targetType == ActionState::LOOKING_FOR_DOG ? RED : BLUE;
            marker.color.a = 1;
            lookDirectionPub.publish(marker);
        }
    }

    void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
        ROS_DEBUG(
                "Received dog position callback. Unknown is %u and stale is %u and stamp is %f and measured time is %f. State is %s",
                dogPosition->unknown, dogPosition->stale, dogPosition->header.stamp.toSec(),
                dogPosition->measuredTime.toSec(), STATE_NAMES[static_cast<int>(state)].c_str());

        if (dogPosition->unknown || dogPosition->stale) {
            // Calculate the score
            double score = searchForDogWeight
                    * (ros::Time::now().toSec() - dogPosition->measuredTime.toSec());
            ROS_DEBUG("Calculated score for dog search of %f based on weight %f and time %f", score,
                    searchForDogWeight,
                    ros::Time::now().toSec() - dogPosition->measuredTime.toSec());

            PointStamped position;
            position.point = dogPosition->pose.pose.position;
            position.header = dogPosition->pose.header;
            assert(
                    (dogPosition->unknown || position.header.frame_id.size() > 0)
                    && "frame_id was not set");
            tryGoal(ActionState::LOOKING_FOR_DOG, score, position, !dogPosition->unknown);
            return;
        }

        if (state == ActionState::LOOKING_FOR_DOG) {
            ROS_INFO(
                    "Search for dog located the dog. Unknown is %u and stale is %u and stamp is %f and measured time is %f. State is %s",
                    dogPosition->unknown, dogPosition->stale, dogPosition->header.stamp.toSec(),
                    dogPosition->measuredTime.toSec(),
                    STATE_NAMES[static_cast<int>(state)].c_str());

            pointHeadClient.cancelGoal();
            pointArmClient.cancelGoal();
            resetState();
            resetSearch();
            unsubscribeToPoints();
            ROS_DEBUG("Exiting dog position callback after dog found");
        }
    }

    void pathViewCallback(const PathViewInfoConstPtr& pathView) {
        ROS_DEBUG("Received path view callback. Ratio is %f and stamp is %f. State is %s",
                pathView->visibilityRatio, pathView->header.stamp.toSec(),
                STATE_NAMES[static_cast<int>(state)].c_str());

        if (pathView->visibilityRatio < pathVisibilityThreshold) {
            double score = lookAtPathWeight * (1.0 - pathView->visibilityRatio) * 100.0
                    * (ros::Time::now().toSec() - pathView->measuredTime.toSec());
            ROS_DEBUG(
                    "Calculated score for look at path of %f based on weight %f and ratio %f and duration %f",
                    score, lookAtPathWeight, pathView->visibilityRatio,
                    ros::Time::now().toSec() - pathView->measuredTime.toSec());
            tryGoal(ActionState::LOOKING_AT_PATH, score, pathView->center, true);
            return;
        }

        if (state == ActionState::LOOKING_AT_PATH) {
            ROS_INFO("Look at path completed. Ratio is %f and stamp is %f and State is %s",
                    pathView->visibilityRatio, pathView->header.stamp.toSec(),
                    STATE_NAMES[static_cast<int>(state)].c_str());

            pointHeadClient.cancelGoal();
            pointArmClient.cancelGoal();
            resetState();
            ROS_DEBUG("Exiting path view callback after path visible");
        }
    }

    void searchedPointsCB(const sensor_msgs::PointCloud2ConstPtr points) {

        if (searchCloud->points.size() == 0 || state != ActionState::LOOKING_FOR_DOG) {
            ROS_DEBUG("Ignoring point cloud. Not currently executing a search");
            return;
        }

        ROS_DEBUG("Received %lu searched points in frame %s", points->data.size(),
                points->header.frame_id.c_str());
        if (points->data.size() == 0) {
            return;
        }

        if (!tf.waitForTransform(points->header.frame_id, "/base_footprint", points->header.stamp,
                ros::Duration(0.1))) {
            ROS_WARN("Failed to get transform from %s to /base_footprint",
                    points->header.frame_id.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*points, *cloud);
        pcl_ros::transformPointCloud("/base_footprint", *cloud, *cloud, tf);

        // Clear the z-indexes. These should all be near zero.
        for (unsigned int i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].z = 0.0;
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);
        size_t initialPoints = searchCloud->points.size();
        ROS_DEBUG("Prior to filtering the cloud has %lu points. Filtering with radius %f",
                initialPoints, utils::square(searchCellSize / 2.0));

        searchCloud->points.erase(
                std::remove_if(searchCloud->points.begin(), searchCloud->points.end(),
                        HasPointInCell(cloud, kdtree, utils::square(searchCellSize / 2.0))),
                        searchCloud->points.end());

        size_t remainingPoints = searchCloud->points.size();
        assert(remainingPoints <= initialPoints);
        ROS_DEBUG("Filtering complete. The search cloud has %lu points remaining", remainingPoints);

        ROS_DEBUG("Points update in the %s frame removed %lu points",
                points->header.frame_id.c_str(), initialPoints - remainingPoints);

        // Publish the results
        publishSearchCloud();
    }

    void publishSearchCloud() {
        if (searchCloudPub.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*searchCloud, *output);
            output->header.stamp = ros::Time::now();
            output->header.frame_id = "/base_footprint";
            searchCloudPub.publish(output);
        }
    }

    bool moveToNextSearchState() {
        ROS_DEBUG("Transitioning to next search state. Current state is %s.",
                SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
        if (searchState == SearchState::NONE) {
            // Start a new search
            searchState = SearchState::LAST_KNOWN;
            resetSearchMap();
        }
        else if (searchState == SearchState::LAST_KNOWN) {
            searchState = SearchState::HEAD;
        }
        else if (searchState == SearchState::HEAD) {
            // Attempt to search any points again with the arm.
            searchedPoints.clear();
            searchState = SearchState::ARM;
        }
        else {
            ROS_WARN("Search failed. Resetting search state to retry");
            searchState = SearchState::NONE;
            state = ActionState::IDLE;
            currentActionScore = 0;
        }
        ROS_DEBUG("Transition complete. New search state is %s.",
                SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
        // Return whether the search aborted
        return searchState != SearchState::NONE;
    }

    bool nextBestSearch(geometry_msgs::PointStamped& resultPoint) const {
        ROS_DEBUG("Executing nextBestSearch. Current search state is %s",
                SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
        assert(searchState == SearchState::HEAD || searchState == SearchState::ARM);

        if (searchCloud->points.size() == 0) {
            ROS_WARN("Search cloud is empty");
            resultPoint = handInBaseFrame();
            return true;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(searchCloud);

        vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setSearchMethod(tree);
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(1000);
        ec.setInputCloud(searchCloud);
        ec.extract(clusterIndices);

        // Now find the largest cluster. Sort by size (smallest to largest).
        std::sort(clusterIndices.begin(), clusterIndices.end(), ClusterSmaller());

        // Reverse the order so the largest is first.
        std::reverse(clusterIndices.begin(), clusterIndices.end());

        // Search through the clusters in increasing order until we find one that has not been searched.
        Eigen::Vector4f centroid;
        vector<pcl::PointIndices>::const_iterator it;
        for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            ROS_DEBUG("Selected a next best search cluster with size %lu", it->indices.size());

            // Now find the centroid
            pcl::compute3DCentroid(*searchCloud, it->indices, centroid);

            // Convert the centroid to a point stamped
            resultPoint.header.frame_id = "/base_footprint";
            resultPoint.header.stamp = ros::Time::now();

            // Convert the centroid to a geometry msg point
            resultPoint.point.x = centroid[0];
            resultPoint.point.y = centroid[1];
            resultPoint.point.z = centroid[2];

            // Check how close it is to a searched point
            PointStampedVector::const_iterator closest = std::min_element(searchedPoints.begin(),
                    searchedPoints.end(), DistanceFromPoint(resultPoint));
            if (closest == searchedPoints.end()) {
                ROS_DEBUG("No searched points to check for closeness.");
                break;
            }
            if (utils::pointToPointXYDistance(closest->point, resultPoint.point) > 0.01) {
                ROS_DEBUG("Selected a point that was %f distance away from the closest point",
                        utils::pointToPointXYDistance(closest->point, resultPoint.point));
                break;
            }
            else {
                ROS_DEBUG("Target at %f, %f, %f was not selected because it was too close to searched point %f, %f, %f",
                        resultPoint.point.x, resultPoint.point.y, resultPoint.point.z, closest->point.x, closest->point.y, closest->point.z);
            }
        }

        if (it == clusterIndices.end()) {
            ROS_DEBUG("Failed to find a cluster to search");
            return false;
        }

        return true;
    }

protected:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    PointHeadClient pointHeadClient;
    PointArmClient pointArmClient;

    tf::TransformListener tf;

    ActionState state;
    SearchState searchState;
    ros::Timer timeout;
    double leashLength;
    double pathVisibilityThreshold;
    double lookAtPathWeight;
    double searchForDogWeight;

    double currentActionScore;
    double searchCellSize;
    double clusterTolerance;
    unsigned int currentSearchOriginalSize;

    PointStampedVector searchedPoints;

    //! Track number of interrupts for metrics
    unsigned int interrupts;

    //! Publisher for the look direction
    ros::Publisher lookDirectionPub;

    //! Dog position subscriber
    unique_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

    //! Path view subscriber
    unique_ptr<message_filters::Subscriber<PathViewInfo> > pathViewSub;

    //! Search points filter subscription for the wide stereo
    unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > headSearchPointsSub;

    //! Search points filter subscription for the arm
    unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > armSearchPointsSub;

    //! Search cloud publisher
    ros::Publisher searchCloudPub;

    //! Points to search
    pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "focus_head_action");
    FocusHead node;
    ros::spin();
    return 0;
}
