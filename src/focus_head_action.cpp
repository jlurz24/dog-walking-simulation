#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>
#include <type_traits>

// Generated messages
#include <dogsim/FocusHeadAction.h>
#include <dogsim/FocusHeadGoal.h>

namespace {
using namespace dogsim;
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::ActionServer<FocusHeadAction> FocusHeadActionServer;

struct XYOffset {
    double x;
    double y;
};

/**
 * Array that instructs how to search
 */
static const XYOffset searchOffsets[] = { {0, 0}, {0, 0}, {1, 0}, {0, -1}, {-1, 0}, {0, 1} };

static const ros::Duration FOCUS_TIMEOUT(1.0);

static const string STATE_NAMES[] = {"Idle", "Looking_At_Path", "Looking_for_Dog"};
static const string SEARCH_STATE_NAMES[] = {"None", "Center", "Top", "Left", "Bottom", "Right", "Done"};

class FocusHead {
private:
    enum class ActionState {
        IDLE, LOOKING_AT_PATH, LOOKING_FOR_DOG
    };

    enum class SearchState {
        NONE,
        CENTER,
        TOP,
        LEFT,
        BOTTOM,
        RIGHT,
        DONE
    };

public:
    FocusHead(const string& name) :
        as(nh, name, false), actionName(name),
        pointHeadClient("/head_traj_controller/point_head_action", true), state(ActionState::IDLE), searchState(SearchState::NONE) {

        as.registerGoalCallback(boost::bind(&FocusHead::goalCallback, this, _1));
        as.registerCancelCallback(boost::bind(&FocusHead::cancelCallback, this, _1));
        nh.param("leash_length", leashLength, 2.0);

        dogPositionSub.reset(
                new message_filters::Subscriber<DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogPositionSub->registerCallback(boost::bind(&FocusHead::dogPositionCallback, this, _1));
        dogPositionSub->unsubscribe();
        pointHeadClient.waitForServer();

        lookDirectionPub = nh.advertise<visualization_msgs::Marker>(
                "/focus_head_action/look_direction_viz", 1);

        as.start();
    }

private:

    void cancelCallback(FocusHeadActionServer::GoalHandle& gh) {
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_DEBUG("Canceling the focus head action");
        // See if our current goal is the one that needs to be canceled
        if (currentGH != gh) {
            ROS_DEBUG("Ignoring cancel request for unknown goal");
            return;
        }
        clearState();
        searchState = SearchState::NONE;
        dogsim::FocusHeadResultPtr result(new FocusHeadResult);
        result->actionCompleted = false;
        currentGH.setCanceled(*result, "Action was cancelled");
        pointHeadClient.cancelGoal();
    }

    void timeoutCallback(const ros::TimerEvent& e){
        ROS_INFO("Point head action timed out");

        boost::mutex::scoped_lock lock(stateMutex);

        // Ignore any timeouts after completion.
        if(state == ActionState::IDLE){
            ROS_INFO("Timeout received when state is idle");
            return;
        }

        if(state == ActionState::LOOKING_FOR_DOG){
            ROS_INFO("Continuing search after timeout");
            if(execute(PointStamped(), FocusHeadGoal::DOG_TARGET, false)){
                return;
            }
            ROS_INFO("Failed to schedule next search location. Search may have completed.");
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = true;
            currentGH.setAborted(*result, "Search for dog failed");
        }
        else {
            assert(state == ActionState::LOOKING_AT_PATH);
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = false;
            currentGH.setAborted(*result, "Point head action timed out");
        }

        stateMutex.unlock();
        pointHeadClient.cancelGoal();
        stateMutex.lock();
        clearState();
    }

    bool goalCallback(FocusHeadActionServer::GoalHandle gh) {
        ROS_DEBUG("Executing goal callback for FocusHeadAction");

        boost::mutex::scoped_lock lock(stateMutex);

        if (state == ActionState::LOOKING_AT_PATH && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Rejecting dog target because currently looking at path");
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = false;
            gh.setRejected(*result, "Rejecting dog target because currently looking at path");
            return false;
        }
        if (state == ActionState::LOOKING_FOR_DOG && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Ignoring additional request to look for dog");
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = false;
            gh.setRejected(*result, "Ignoring additional request to look for dog");
            return false;
        }
        // Don't want to preempt looking at path because we could end up in a state where we never
        // finish looking at the path
        if (state == ActionState::LOOKING_AT_PATH && gh.getGoal()->target == FocusHeadGoal::PATH_TARGET) {
            ROS_DEBUG("Ignoring additional request to look at path");
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = false;
            gh.setRejected(*result, "Ignoring additional request to look at path");
            return false;
        }

        if (state != ActionState::IDLE) {
            ROS_INFO("Preempting dog search focus head target for new goal");
            assert(state == ActionState::LOOKING_FOR_DOG);
            state = ActionState::IDLE;
            // Do not reset search state as we want the search to continue when it is
            // resumed.
            dogPositionSub->unsubscribe();
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = false;
            currentGH.setCanceled(*result, "Search for dog was preempted by a look at path request");
            stateMutex.unlock();
            pointHeadClient.cancelGoal();
            stateMutex.lock();
        }

        ROS_INFO("Execution goal accepted for target: %s", gh.getGoal()->target.c_str());
        currentGH = gh;
        currentGH.setAccepted();

        // Clear the search state if it completed.
        // TODO: Move this to a more appropriate location.
        if(searchState == SearchState::DONE){
            ROS_INFO("Resetting search state");
            searchState = SearchState::NONE;
        }

        if(!execute(currentGH.getGoal()->position, currentGH.getGoal()->target, currentGH.getGoal()->isPositionSet)){
            ROS_INFO("Failed to schedule point head action. Search completed unsuccessfully.");
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = true;
            currentGH.setAborted(*result, "Search completed unsuccessfully");
            searchState = SearchState::NONE;
            clearState();
            return false;
        }
        ROS_DEBUG("Execute goal callback completed");
        return true;
    }

    PointStamped handInBaseFrame() const {
        // Use the current right hand position at height 0 as the starting point.
        // Determine the position of the hand in the base frame.
        PointStamped handInBaseFrame;
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "r_wrist_roll_link";
        tf.waitForTransform("/base_footprint", handInHandFrame.header.frame_id, ros::Time(0), ros::Duration(5.0));
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame,
                    handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR(
                    "Failed to transform hand position to /base_footprint");
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
        tf.waitForTransform(requestedFrame, cameraInCameraFrame.header.frame_id, ros::Time(0), ros::Duration(5.0));

        try {
            tf.transformPoint(requestedFrame, ros::Time(0), cameraInCameraFrame,
                    cameraInCameraFrame.header.frame_id, cameraInRequestedFrame);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR(
                    "Failed to transform camera position to %s", requestedFrame.c_str());
            throw ex;
        }
        cameraInRequestedFrame.header.frame_id = requestedFrame;
        cameraInRequestedFrame.header.stamp = ros::Time::now();
        return cameraInRequestedFrame;
    }

    void pointHeadCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const pr2_controllers_msgs::PointHeadResultConstPtr result){

        ROS_INFO("Received point head complete callback");
        boost::mutex::scoped_lock lock(stateMutex);
        if(state == ActionState::IDLE){
            // Spurious late callback.
            return;
        }

        timeout.stop();
        if(state == ActionState::LOOKING_FOR_DOG){
            if(!execute(PointStamped(), FocusHeadGoal::DOG_TARGET, false)){
                ROS_INFO("Failed to schedule next search location. Search may have completed.");
                dogsim::FocusHeadResultPtr result(new FocusHeadResult);
                result->actionCompleted = true;
                currentGH.setAborted(*result, "Search completed unsuccessfully");
                searchState = SearchState::NONE;
                clearState();
            }
            else {
                ROS_DEBUG("Continuing search after successfully pointing head.");
            }
        }
        // Looking at path currently.
        else {
            assert(state == ActionState::LOOKING_AT_PATH);
            dogsim::FocusHeadResultPtr result(new FocusHeadResult);
            result->actionCompleted = true;
            currentGH.setSucceeded(*result, "Looked at path completed successfully");
            clearState();
        }
    }

    void clearState(){
        dogPositionSub->unsubscribe();
        state = ActionState::IDLE;
    }

    bool execute(const PointStamped& target, const string& targetType, const bool isPositionSet) {
        pr2_controllers_msgs::PointHeadGoal phGoal;
        if (targetType == FocusHeadGoal::DOG_TARGET) {
            state = ActionState::LOOKING_FOR_DOG;
            // If the last position is known, check that location.
            if (isPositionSet) {
                ROS_INFO("Searching last known position");
                phGoal.target = target;
                // Last known position is not a search and therefore has no search state.
            }
            // Execute a search
            else {
                // This may increment the search step if the last movement
                // was interrupted, but accept that as most of the movement likely
                // completed.
                int nextStep = static_cast<int>(searchState) + 1;
                searchState = static_cast<SearchState>(nextStep);
                ROS_INFO("Executing search step %s", SEARCH_STATE_NAMES[nextStep].c_str());
                // TODO: Make sure all search states are being executed.
                if(searchState == SearchState::DONE){
                    ROS_INFO("No more search steps to execute");
                    return false;
                }

                PointStamped handPosition = handInBaseFrame();
                // Calculate the x distance.
                double planarLeashLength = sqrt(utils::square(leashLength) - utils::square(handPosition.point.z));
                phGoal.target.header = handPosition.header;
                phGoal.target.point.z = 0;
                phGoal.target.point.x = handPosition.point.x + planarLeashLength * searchOffsets[nextStep].x;
                phGoal.target.point.y = handPosition.point.y + planarLeashLength * searchOffsets[nextStep].y;
                ROS_INFO("Search target - x: %f, y: %f, z: %f", phGoal.target.point.x, phGoal.target.point.y, phGoal.target.point.z);
            }
            dogPositionSub->subscribe();
            ROS_DEBUG("Subscribed to the dog position");
        }
        else if (targetType == FocusHeadGoal::PATH_TARGET) {
            state = ActionState::LOOKING_AT_PATH;
            ROS_INFO("Focusing head on path target in frame %s, %f %f %f", target.header.frame_id.c_str(), target.point.x, target.point.y, target.point.z);
            phGoal.target = target;
            phGoal.target.point.z = 0;
        }
        else {
            ROS_ERROR("Unknown focus target type: %s. Options are[%s, %s]", targetType.c_str(), FocusHeadGoal::DOG_TARGET.c_str(), FocusHeadGoal::PATH_TARGET.c_str());
            return false;
        }

        phGoal.pointing_frame = "wide_stereo_link";
        // Pointing axis defaults to the x-axis.

        ROS_INFO("Sending goal of type %s to point head client", targetType.c_str());

        visualizeGoal(phGoal.target, targetType);
        stateMutex.unlock();
        pointHeadClient.sendGoal(phGoal,
                boost::bind(&FocusHead::pointHeadCompleteCallback, this, _1, _2));
        stateMutex.lock();
        timeout = nh.createTimer(FOCUS_TIMEOUT, &FocusHead::timeoutCallback, this,
                true /* One shot */);
        ROS_INFO("Focus target step completed");
        return true;
    }

    void visualizeGoal(const PointStamped& goal, const string& targetType) const {
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
            marker.color = targetType == FocusHeadGoal::DOG_TARGET ? RED : BLUE;
            marker.color.a = 1;
            lookDirectionPub.publish(marker);
        }
    }

    void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
        ROS_DEBUG("Received dog position callback. Unknown is %u and stale is %u and stamp is %f and measured time is %f. State is %s",
                dogPosition->unknown, dogPosition->stale, dogPosition->header.stamp.toSec(), dogPosition->measuredTime.toSec(), STATE_NAMES[static_cast<int>(state)].c_str());

        if(dogPosition->unknown || dogPosition->stale || state != ActionState::LOOKING_FOR_DOG){
            return;
        }

        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Search for dog located the dog. Unknown is %u and stale is %u and stamp is %f and measured time is %f. State is %s",
                dogPosition->unknown, dogPosition->stale, dogPosition->header.stamp.toSec(), dogPosition->measuredTime.toSec(), STATE_NAMES[static_cast<int>(state)].c_str());

        dogsim::FocusHeadResultPtr result(new FocusHeadResult);
        result->actionCompleted = true;
        currentGH.setSucceeded(*result, "Search located dog");
        stateMutex.unlock();
        pointHeadClient.cancelGoal();
        stateMutex.lock();
        searchState = SearchState::NONE;
        clearState();
        ROS_INFO("Exiting dog position callback after dog found");
    }

protected:
    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::ActionServer<dogsim::FocusHeadAction> as;
    string actionName;
    PointHeadClient pointHeadClient;
    tf::TransformListener tf;
    boost::mutex stateMutex;
    FocusHeadActionServer::GoalHandle currentGH;
    ActionState state;
    SearchState searchState;
    ros::Timer timeout;
    double leashLength;

    //! Publisher for the look direction
    ros::Publisher lookDirectionPub;

    //! Dog position subscriber
    auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "focus_head_action");
    FocusHead action(ros::this_node::getName());
    ros::spin();
    return 0;
}
