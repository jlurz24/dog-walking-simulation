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

static const ros::Duration FOCUS_TIMEOUT(2.0);

class FocusHead {
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
        currentGH.setCanceled();
        pointHeadClient.cancelGoal();
    }

    void timeoutCallback(const ros::TimerEvent& e){
        boost::mutex::scoped_lock lock(stateMutex);

        // Ignore any timeouts after completion
        if(state == ActionState::IDLE){
            ROS_DEBUG("Timeout received when state is idle");
            return;
        }

        if(state == ActionState::LOOKING_FOR_DOG && execute(PointStamped(), FocusHeadGoal::DOG_TARGET, false)){
            ROS_DEBUG("Continuing search after timeout");
            return;
        }

        currentGH.setAborted();
        pointHeadClient.cancelGoal();
        searchState = SearchState::NONE;
        clearState();
    }

    bool goalCallback(FocusHeadActionServer::GoalHandle gh) {
        ROS_DEBUG("Executing goal callback for FocusHeadAction");

        boost::mutex::scoped_lock lock(stateMutex);

        if (state == ActionState::LOOKING_AT_PATH && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Rejecting dog target because currently looking at path");
            gh.setRejected();
            return false;
        }
        if (state == ActionState::LOOKING_FOR_DOG && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Ignoring additional request to look for dog");
            gh.setRejected();
            return false;
        }

        if (state != ActionState::IDLE) {
            ROS_INFO("Preempting current focus head target for new goal");
            assert(state == ActionState::LOOKING_AT_PATH || state == ActionState::LOOKING_FOR_DOG);
            state = ActionState::IDLE;
            // Do not reset search state as we want the search to continue when it is
            // resumed.
            currentGH.setCanceled();
            pointHeadClient.cancelGoal();
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
            ROS_ERROR("Failed to schedule point head action");
            currentGH.setAborted();
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

    void pointHeadCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const pr2_controllers_msgs::PointHeadResultConstPtr result){
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Received point head complete callback");

        if(state == ActionState::IDLE){
            // Spurious late callback.
            return;
        }

        if(state == ActionState::LOOKING_FOR_DOG){
            if(!execute(PointStamped(), FocusHeadGoal::DOG_TARGET, false)){
                ROS_DEBUG("Failed to schedule next search location. Search may have completed.");
                currentGH.setAborted();
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
            currentGH.setSucceeded();
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
                ROS_INFO("Executing search step %i", searchState);
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

        ROS_INFO("Sending goal to point head client");
        pointHeadClient.sendGoal(phGoal,
                boost::bind(&FocusHead::pointHeadCompleteCallback, this, _1, _2));
        timeout = nh.createTimer(FOCUS_TIMEOUT, &FocusHead::timeoutCallback, this,
                true /* One shot */);
        ROS_INFO("Focus target step completed");
        return true;
    }

    void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
        ROS_INFO("Received dog position callback. Stamp is %f and measured time is %f. State is %u", dogPosition->header.stamp.toSec(), dogPosition->measuredTime.toSec(), state);
        if(dogPosition->unknown || state != ActionState::LOOKING_FOR_DOG || dogPosition->header.stamp != dogPosition->measuredTime){
            return;
        }
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Search for dog located the dog");

        currentGH.setSucceeded();
        pointHeadClient.cancelGoal();
        searchState = SearchState::NONE;
        clearState();
        ROS_INFO("Exiting dog position callback");
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
