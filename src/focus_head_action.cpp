#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>
#include <type_traits>

// Generated messages
#include <dogsim/DogSearchFailed.h>
#include <dogsim/PathViewInfo.h>

namespace {
using namespace dogsim;
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

struct XYOffset {
    double x;
    double y;
};

/**
 * Array that instructs how to search
 */
static const XYOffset searchOffsets[] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { 0, 1 } };

static const ros::Duration FOCUS_TIMEOUT(1.0);
static const double PATH_VIS_THRESHOLD_DEFAULT = 0.99;
static const double LOOK_AT_PATH_WEIGHT_DEFAULT = 15;
static const double SEARCH_FOR_DOG_WEIGHT_DEFAULT = 1;
static const string STATE_NAMES[] = { "Idle", "Looking_At_Path", "Looking_for_Dog" };
static const string SEARCH_STATE_NAMES[] = { "None", "Last Known", "Center", "Top", "Left", "Bottom", "Right",
        "Done" };

class FocusHead {
private:
    enum class ActionState {
        IDLE, LOOKING_AT_PATH, LOOKING_FOR_DOG
    };

    enum class SearchState {
        NONE, LAST_KNOWN, CENTER, TOP, LEFT, BOTTOM, RIGHT, DONE
    };

public:
    FocusHead() :
            pnh("~"),
            pointHeadClient("/head_traj_controller/point_head_action", true),
            state(ActionState::IDLE),
            searchState(SearchState::NONE),
            currentActionScore(0),
            interrupts(0){

        nh.param("leash_length", leashLength, 2.0);
        nh.param("path_visibility_threshold", pathVisibilityThreshold, PATH_VIS_THRESHOLD_DEFAULT);
        pnh.param("look_at_path_weight", lookAtPathWeight, LOOK_AT_PATH_WEIGHT_DEFAULT);
        pnh.param("search_for_dog_weight", searchForDogWeight, SEARCH_FOR_DOG_WEIGHT_DEFAULT);

        dogPositionSub.reset(
                new message_filters::Subscriber<DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogPositionSub->registerCallback(boost::bind(&FocusHead::dogPositionCallback, this, _1));

        pathViewSub.reset(
                new message_filters::Subscriber<PathViewInfo>(nh, "/path_visibility_detector/view",
                        1));
        pathViewSub->registerCallback(boost::bind(&FocusHead::pathViewCallback, this, _1));

        pointHeadClient.waitForServer();

        lookDirectionPub = nh.advertise<visualization_msgs::Marker>(
                "/focus_head_action/look_direction_viz", 1);

        dogSearchFailedPub = nh.advertise<DogSearchFailed>("/focus_head_action/dog_search_failed",
                1);
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
            if (execute(PointStamped(), ActionState::LOOKING_FOR_DOG, false)) {
                return;
            }

            ROS_WARN("Failed to schedule next search location. Search may have completed.");
            DogSearchFailedConstPtr msg(new DogSearchFailed());
            dogSearchFailedPub.publish(msg);
        }
        else {
            assert(state == ActionState::LOOKING_AT_PATH);
        }

        pointHeadClient.cancelGoal();
        state = ActionState::IDLE;
        currentActionScore = 0;
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

        // Clear the search state if it completed.
        if (searchState == SearchState::DONE) {
            ROS_INFO("Resetting search state");
            searchState = SearchState::NONE;
        }

        if (!execute(position, actionType, isPositionSet)) {
            ROS_WARN("Failed to schedule point head action. Search completed unsuccessfully.");
            searchState = SearchState::NONE;
            state = ActionState::IDLE;
            currentActionScore = 0;

            DogSearchFailedConstPtr msg(new DogSearchFailed());
            dogSearchFailedPub.publish(msg);
            return false;
        }

        return true;
    }

    PointStamped handInBaseFrame() const {
        // Use the current right hand position at height 0 as the starting point.
        // Determine the position of the hand in the base frame.
        PointStamped handInBaseFrame;
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "r_wrist_roll_link";
        tf.waitForTransform("/base_footprint", handInHandFrame.header.frame_id, ros::Time(0),
                ros::Duration(30.0));
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
            if (!execute(PointStamped(), ActionState::LOOKING_FOR_DOG, false)) {
                ROS_WARN("Failed to schedule next search location. Search may have completed.");
                searchState = SearchState::NONE;
                state = ActionState::IDLE;
                currentActionScore = 0;
                DogSearchFailedConstPtr msg(new DogSearchFailed());
                dogSearchFailedPub.publish(msg);
            }
            else {
                ROS_DEBUG("Continuing search after successfully pointing head.");
            }
        }
        // Looking at path currently.
        else {
            assert(state == ActionState::LOOKING_AT_PATH);
            state = ActionState::IDLE;
            currentActionScore = 0;
        }
    }

    bool execute(const PointStamped& target, const ActionState targetType, const bool isPositionSet) {
        ROS_INFO("Executing search for target type %s with position set %u and target frame %s",
                STATE_NAMES[static_cast<int>(targetType)].c_str(), isPositionSet, target.header.frame_id.c_str());

        assert(!isPositionSet || target.header.frame_id.size() > 0);
        pr2_controllers_msgs::PointHeadGoal phGoal;
        if (targetType == ActionState::LOOKING_FOR_DOG) {
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
                if (searchState == SearchState::DONE) {
                    ROS_INFO("No more search steps to execute");
                    return false;
                }

                PointStamped handPosition = handInBaseFrame();
                // Calculate the x distance.
                double planarLeashLength = sqrt(
                        utils::square(leashLength) - utils::square(handPosition.point.z));
                phGoal.target.header = handPosition.header;
                phGoal.target.point.z = 0;
                phGoal.target.point.x = handPosition.point.x
                        + planarLeashLength * searchOffsets[static_cast<int>(searchState)].x;
                phGoal.target.point.y = handPosition.point.y
                        + planarLeashLength * searchOffsets[static_cast<int>(searchState)].y;
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
        return true;
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
            searchState = SearchState::NONE;
            state = ActionState::IDLE;
            currentActionScore = 0;
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
            searchState = SearchState::NONE;
            state = ActionState::IDLE;
            currentActionScore = 0;
            ROS_INFO("Exiting path view callback after path visible");
        }
    }

    void moveToNextSearchState() {
        int nextStep = static_cast<int>(searchState) + 1;
        searchState = static_cast<SearchState>(nextStep);
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
    
    //! Track number of interupts for metrics
    unsigned int interrupts;

    //! Publisher to notify that the dog search failed
    ros::Publisher dogSearchFailedPub;

    //! Publisher for the look direction
    ros::Publisher lookDirectionPub;

    //! Dog position subscriber
    auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

    //! Path view subscriber
    auto_ptr<message_filters::Subscriber<PathViewInfo> > pathViewSub;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "focus_head_action");
    FocusHead node;
    ros::spin();
    return 0;
}
