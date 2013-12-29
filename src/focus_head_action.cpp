#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>
#include <type_traits>
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

// Generated messages
#include <dogsim/DogSearchFailed.h>
#include <dogsim/PathViewInfo.h>

namespace {
using namespace dogsim;
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

static const ros::Duration FOCUS_TIMEOUT(1.0);
static const double PATH_VIS_THRESHOLD_DEFAULT = 0.99;
static const double LOOK_AT_PATH_WEIGHT_DEFAULT = 10;
static const double SEARCH_FOR_DOG_WEIGHT_DEFAULT = 1;
static const string STATE_NAMES[] = { "Idle", "Looking_At_Path", "Looking_for_Dog" };
static const string SEARCH_STATE_NAMES[] = { "None", "Last Known", "Head", "Arm", "Done" };
static const double SEARCH_CELL_SIZE_DEFAULT = 0.05;
static const double BASE_RADIUS = 0.668 / 2.0;
static const double CLUSTER_TOLERANCE_DEFAULT = 0.5;
static const int POINT_UPDATE_THRESHOLD_DEFAULT = 5;

static Eigen::Vector3f operator-(const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs){
  return Eigen::Vector3f(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

static bool isInCell(const pcl::PointXYZ& cellCenter, const pcl::PointXYZ& p, const double searchCellSize) {
     ROS_INFO("Checking if cell @ center %f %f contains point %f %f with cell size %f", cellCenter.x, cellCenter.y, p.x, p.y, searchCellSize);
    bool result = (p.x <= cellCenter.x + searchCellSize / 2.0 && p.x >= cellCenter.x - searchCellSize / 2.0) &&
            (p.y <= cellCenter.y + searchCellSize / 2.0 && p.y >= cellCenter.y - searchCellSize / 2.0);
     ROS_INFO("Returning %u", result);
     return result;
}

struct HasPointInCell {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree;
    const double searchCellSize;


    HasPointInCell(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr aCloud, const pcl::KdTreeFLANN<pcl::PointXYZ>& aKdtree, const double aSearchCellSize) :
            cloud(aCloud), kdtree(aKdtree), searchCellSize(aSearchCellSize) {
    }

    bool operator()(const pcl::PointXYZ& p) {
        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);

        if(kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) == 0){
          ROS_INFO("No neighboring point in cloud");
          return false;
        }
        
        return isInCell(p, cloud->points[pointIdxNKNSearch[0]], searchCellSize);
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
        pnh("~"),
        pointHeadClient("/head_traj_controller/point_head_action", true),
        state(ActionState::IDLE),
        searchState(SearchState::NONE),
        currentActionScore(0),
        interrupts(0),
        searchCloud(new pcl::PointCloud<pcl::PointXYZ>()){

        nh.param("leash_length", leashLength, 2.0);
        nh.param("path_visibility_threshold", pathVisibilityThreshold, PATH_VIS_THRESHOLD_DEFAULT);
        pnh.param("look_at_path_weight", lookAtPathWeight, LOOK_AT_PATH_WEIGHT_DEFAULT);
        pnh.param("search_for_dog_weight", searchForDogWeight, SEARCH_FOR_DOG_WEIGHT_DEFAULT);
        pnh.param("search_cell_size", searchCellSize, SEARCH_CELL_SIZE_DEFAULT);
        pnh.param("cluster_tolerance", clusterTolerance, CLUSTER_TOLERANCE_DEFAULT);
        pnh.param("point_update_threshold", pointUpdateThreshold, POINT_UPDATE_THRESHOLD_DEFAULT);

        dogPositionSub.reset(
                new message_filters::Subscriber<DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogPositionSub->registerCallback(boost::bind(&FocusHead::dogPositionCallback, this, _1));

        pathViewSub.reset(
                new message_filters::Subscriber<PathViewInfo>(nh, "/path_visibility_detector/view",
                        1));
        pathViewSub->registerCallback(boost::bind(&FocusHead::pathViewCallback, this, _1));

        searchPointsSub.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, /*"search_cloud_in"*/"sensor_search_cloud",
                        1));
        searchPointsSub->registerCallback(boost::bind(&FocusHead::searchedPointsCB, this, _1));

        pointHeadClient.waitForServer();

        lookDirectionPub = nh.advertise<visualization_msgs::Marker>(
                "/focus_head_action/look_direction_viz", 1);

        dogSearchFailedPub = nh.advertise<DogSearchFailed>("/focus_head_action/dog_search_failed",
                1);

        searchCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/focus_head_action/search_cloud", 1);
    }

private:

    void timeoutCallback(const ros::TimerEvent& e) {
        ROS_INFO("Point head action timed out");

        // Ignore any timeouts after completion.
        if (state == ActionState::IDLE) {
            ROS_INFO("Timeout received when state is idle");
            return;
        }

        if (state == ActionState::LOOKING_FOR_DOG) {
            ROS_INFO("Continuing search after timeout");
            executeNext(PointStamped(), ActionState::LOOKING_FOR_DOG, false);
            return;
        }

        assert(state == ActionState::LOOKING_AT_PATH);
        pointHeadClient.cancelGoal();
        resetState();
    }

    bool tryGoal(const ActionState actionType, const double score, const PointStamped position, bool isPositionSet) {
        ROS_DEBUG("Received a new goal of type %s with score %f and current score is %f", STATE_NAMES[static_cast<int>(actionType)].c_str(), score, currentActionScore);

        // Don't want to preempt because we could end up in a state where we never
        // finish looking at the path
        if (state == actionType) {
            ROS_DEBUG("Ignoring additional request for current action");
            return false;
        }

        if(score < currentActionScore){
            ROS_DEBUG("Score for new action does not exceed current action");
            return false;
        }

        ROS_INFO("Accepted new goal with score %f (previous score was %f) and type %s",
                score, currentActionScore, STATE_NAMES[static_cast<int>(actionType)].c_str());
        currentActionScore = score;

        if (state != ActionState::IDLE) {
            ROS_INFO("Preempting action for new goal. Total interrupts: %u", interrupts);
            // Do not reset search state as we want the search to continue when it is
            // resumed.
            interrupts++;
            pointHeadClient.cancelGoal();
        }

        state = actionType;

        executeNext(position, actionType, isPositionSet);
        return true;
    }

    void resetSearch(){
        searchState = SearchState::NONE;
    }

    void resetState(){
        state = ActionState::IDLE;
        currentActionScore = 0;
    }

    void resetSearchMap(){
        // Generate a new search cloud.
        searchCloud->points.clear();
        const Point handPosition = handInBaseFrame().point;
        
        double sqPlanarLeashLength = utils::square(leashLength) - utils::square(handPosition.z); 
        double planarLeashLength = sqrt(sqPlanarLeashLength);
        const pcl::PointXYZ planarHand(handPosition.x, handPosition.y, 0);

        ROS_INFO("Creating a new search cloud for planar leash length %f", planarLeashLength);
        unsigned int size = static_cast<unsigned int>(ceil(planarLeashLength * 2 / searchCellSize));
        // The initial point is the bottom left corner.
        double x = handPosition.x - size / 2.0 * searchCellSize;
        for(unsigned int i = 0; i < size; ++i){
            double y = handPosition.y - size / 2.0 * searchCellSize;
            for(unsigned int j = 0; j < size; ++j){
                pcl::PointXYZ p(x, y, 0);
                // Check if the point is inside the base
                if(fabs(x) < BASE_RADIUS && fabs(y) < BASE_RADIUS){
                    ROS_DEBUG("Skipping point inside base @ %f %f", x, y);
                }
                // Check if the point is outside the radius of the leash
                else if(pcl::geometry::squaredDistance(p, planarHand) > sqPlanarLeashLength){
                   ROS_DEBUG("Skipping point outside circle");
                }
                else {
                  searchCloud->points.push_back(p);
                }
                y += searchCellSize;
            }
            x += searchCellSize;
        }
        ROS_INFO("New search map has %lu points", searchCloud->points.size());
    }

    PointStamped handInBaseFrame() const {
        // Use the current right hand position at height 0 as the starting point.
        // Determine the position of the hand in the base frame.
        PointStamped handInBaseFrame;
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "r_wrist_roll_link";
        while(!tf.waitForTransform("/base_footprint", handInHandFrame.header.frame_id, ros::Time(0),
                ros::Duration(30.0))){
            ROS_INFO("Waiting for transform from /r_wrist_roll_link to /base_footprint");
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

    void pointHeadCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const pr2_controllers_msgs::PointHeadResultConstPtr result) {

        ROS_INFO("Received point head complete callback");
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

    void executeNext(const PointStamped& target, const ActionState targetType, const bool isPositionSet) {
        ROS_INFO("Executing search for target type %s with position set %u and target frame %s",
                STATE_NAMES[static_cast<int>(targetType)].c_str(), isPositionSet, target.header.frame_id.c_str());

        assert(!isPositionSet || target.header.frame_id.size() > 0);
        pr2_controllers_msgs::PointHeadGoal phGoal;
        if (targetType == ActionState::LOOKING_FOR_DOG) {
            // Determine if the search failed and should be reset
            if(searchCloud->points.size() == 0){
                ROS_WARN("Search completed but failed to find the dog");
                searchState = SearchState::NONE;
            }

            // This may increment the search step if the last movement
            // was interrupted, but accept that as most of the movement likely
            // completed.
            moveToNextSearchState();

            // If the search state is last known and there is no last known,
            // immediate increment the search.
            if(searchState == SearchState::LAST_KNOWN && !isPositionSet){
                ROS_INFO("Skipping last known search state because position is not set");
                moveToNextSearchState();
            }

            ROS_INFO("Executing search step %s", SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
            if (searchState == SearchState::LAST_KNOWN) {
                ROS_INFO("Searching last known position");
                phGoal.target = target;
            }
            else {
                phGoal.target = nextBestSearch();
                ROS_INFO("Search target - x: %f, y: %f, z: %f", phGoal.target.point.x,
                        phGoal.target.point.y, phGoal.target.point.z);
            }
        }
        else if (targetType == ActionState::LOOKING_AT_PATH) {
            ROS_INFO("Focusing head on path target in frame %s, %f %f %f",
                    target.header.frame_id.c_str(), target.point.x, target.point.y, target.point.z);
            assert(isPositionSet && "is position not set for path action");
            phGoal.target = target;
        }
        else {
            assert(false && "Unknown target type");
        }

        phGoal.pointing_frame = "wide_stereo_link";
        // Pointing axis defaults to the x-axis.

        ROS_INFO("Sending goal of type %s to point head client", STATE_NAMES[static_cast<int>(targetType)].c_str());

        visualizeGoal(phGoal.target, targetType);
        pointHeadClient.sendGoal(phGoal,
                boost::bind(&FocusHead::pointHeadCompleteCallback, this, _1, _2));
        timeout = nh.createTimer(FOCUS_TIMEOUT, &FocusHead::timeoutCallback, this,
                true /* One shot */);
        ROS_INFO("Focus target step completed");
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
            double score = searchForDogWeight * (ros::Time::now().toSec() - dogPosition->measuredTime.toSec());
            ROS_DEBUG("Calculated score for dog search of %f based on weight %f and time %f", score, searchForDogWeight, ros::Time::now().toSec() - dogPosition->measuredTime.toSec());

            // TODO: This used to use the last known but now will only use the
            //       kalman filter position
            PointStamped position;
            position.point = dogPosition->pose.pose.position;
            position.header = dogPosition->pose.header;
            assert((dogPosition->unknown || position.header.frame_id.size() > 0) && "frame_id was not set");
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
            resetState();
            resetSearch();
            ROS_INFO("Exiting dog position callback after dog found");
        }
    }

    void pathViewCallback(const PathViewInfoConstPtr& pathView) {
        ROS_DEBUG(
                "Received path view callback. Ratio is %f and stamp is %f. State is %s",
                pathView->visibilityRatio, pathView->header.stamp.toSec(),
                STATE_NAMES[static_cast<int>(state)].c_str());

        if (pathView->visibilityRatio < pathVisibilityThreshold) {
            double score = lookAtPathWeight * (1.0 - pathView->visibilityRatio) * 100.0 * (ros::Time::now().toSec() - pathView->measuredTime.toSec());
            ROS_DEBUG("Calculated score for look at path of %f based on weight %f and ratio %f and duration %f", score, lookAtPathWeight,
                    pathView->visibilityRatio, ros::Time::now().toSec() - pathView->measuredTime.toSec());
            tryGoal(ActionState::LOOKING_AT_PATH, score, pathView->center, true);
            return;
        }

        if (state == ActionState::LOOKING_AT_PATH) {
            ROS_INFO(
                    "Look at path completed. Ratio is %f and stamp is %f and State is %s",
                    pathView->visibilityRatio, pathView->header.stamp.toSec(),
                    STATE_NAMES[static_cast<int>(state)].c_str());

            pointHeadClient.cancelGoal();
            resetState();
            ROS_INFO("Exiting path view callback after path visible");
        }
    }

    void searchedPointsCB(const sensor_msgs::PointCloud2ConstPtr points){
        // Check for spurious callback.
        if(state != ActionState::LOOKING_FOR_DOG){
            return;
        }

        ROS_INFO("Received searched points in frame %s", points->header.frame_id.c_str());
        if(!tf.waitForTransform(points->header.frame_id, "/base_footprint", points->header.stamp, ros::Duration(0.1))){
            ROS_WARN("Failed to get transform from %s to /base_footprint", points->header.frame_id.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*points, *cloud);
        pcl_ros::transformPointCloud("/base_footprint", *cloud, *cloud, tf);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        vector<int> pointIdxNKNSearch(1);
        vector<float> pointNKNSquaredDistance(1);
        size_t initialPoints = searchCloud->points.size();
        ROS_INFO("Prior to filtering the cloud has %lu points", initialPoints);

        searchCloud->points.erase(std::remove_if(searchCloud->points.begin(), searchCloud->points.end(), HasPointInCell(cloud, kdtree, searchCellSize)), searchCloud->points.end());

        size_t remainingPoints = searchCloud->points.size();
        assert(remainingPoints <= initialPoints);
        ROS_INFO("Filtering complete. The search cloud has %lu points remaining", remainingPoints);

        // TODO: Determine a better way to tell that this was a head message
        // TODO: Determine when to transition between states
        if(searchState == SearchState::HEAD && remainingPoints < 10){
            ROS_INFO("Head search is no longer productive. Only %lu points remaining", remainingPoints);
            moveToNextSearchState();
        }

        if(remainingPoints == 0){
            ROS_WARN("Search failed. Resetting search.");
            searchState = SearchState::NONE;
            moveToNextSearchState();
        }
        
        // Publish the results
        if(searchCloudPub.getNumSubscribers() > 0){
            sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*searchCloud, *output);
            output->header.stamp = ros::Time::now();
            output->header.frame_id = "/base_footprint";
            searchCloudPub.publish(output);
        }
    }

    void moveToNextSearchState() {
        if(searchState == SearchState::NONE){
            // Start a new search
            searchState = SearchState::LAST_KNOWN;
            resetSearchMap();
        }
        else if(searchState == SearchState::LAST_KNOWN){
            searchState = SearchState::HEAD;
        }
        else if(searchState == SearchState::ARM){
            searchState = SearchState::ARM;
            DogSearchFailedConstPtr msg(new DogSearchFailed());
            dogSearchFailedPub.publish(msg);
        }
        else {
            // Nothing to do
        }
    }

    geometry_msgs::PointStamped nextBestSearch() const {
        // Precondition: Search points lock held.
        ROS_INFO("Executing nextBestSearch. Current search state is %s", SEARCH_STATE_NAMES[static_cast<int>(searchState)].c_str());
        assert(searchState == SearchState::HEAD || searchState == SearchState::ARM);

        if(searchCloud->points.size() == 0){
	  ROS_WARN("Search cloud is empty");
          return handInBaseFrame();
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

        // Now find the largest cluster
        unsigned int largestClusterSize = 0;
        vector<pcl::PointIndices>::const_iterator largestCluster;
        for(vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            if(it->indices.size() > largestClusterSize){
                largestClusterSize = it->indices.size();
                largestCluster = it;
            }
        }

        // Now find the centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*searchCloud, largestCluster->indices, centroid);

        // Convert the centroid to a point stamped
        geometry_msgs::PointStamped resultPoint;
        resultPoint.header.frame_id = "/base_footprint";
        resultPoint.header.stamp = ros::Time::now();

        // Convert the centroid to a geometry msg point
        resultPoint.point.x = centroid[0];
        resultPoint.point.y = centroid[1];
        resultPoint.point.z = centroid[2];

        return resultPoint;
    }

protected:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    PointHeadClient pointHeadClient;
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
    int pointUpdateThreshold;

    //! Track number of interrupts for metrics
    unsigned int interrupts;

    //! Publisher to notify that the dog search failed
    ros::Publisher dogSearchFailedPub;

    //! Publisher for the look direction
    ros::Publisher lookDirectionPub;

    //! Dog position subscriber
    unique_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

    //! Path view subscriber
    unique_ptr<message_filters::Subscriber<PathViewInfo> > pathViewSub;

    //! Search points filter subscription for the wide stereo
    unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > searchPointsSub;

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
