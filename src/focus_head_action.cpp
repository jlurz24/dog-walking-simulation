#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>

// Generated messages
#include <dogsim/FocusHeadAction.h>
#include <dogsim/FocusHeadGoal.h>

namespace {
using namespace dogsim;
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::ActionServer<FocusHeadAction> FocusHeadActionServer;

class FocusHead {
    enum ActionState {
        IDLE, LOOKING_AT_PATH, LOOKING_FOR_DOG
    };
public:
    FocusHead(const string& name) :
        as(nh, name, false), actionName(name),
        pointHeadClient("/head_traj_controller/point_head_action", true), state(IDLE) {

        as.registerGoalCallback(boost::bind(&FocusHead::goalCallback, this, _1));
        as.registerCancelCallback(boost::bind(&FocusHead::cancelCallback, this, _1));

        pointHeadClient.waitForServer();
        timeout = nh.createTimer(ros::Duration(1.0), &FocusHead::timeoutCallback, this,
                true /* One shot */);
        timeout.stop();
        as.start();
    }

private:

    void cancelCallback(FocusHeadActionServer::GoalHandle& gh) {
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Canceling the focus head action");
        // See if our current goal is the one that needs to be canceled
        if (currentGH != gh) {
            ROS_INFO("Got a cancel request for some other goal. Ignoring it");
            return;
        }
        currentGH.setCanceled();
        pointHeadClient.cancelGoal();
    }

    void timeoutCallback(const ros::TimerEvent& e){
        boost::mutex::scoped_lock lock(stateMutex);
        // Ignore any timeouts after completion
        if(state == FocusHead::IDLE){
            return;
        }
        ROS_INFO("Received timeout callback");
        currentGH.setAborted();
        pointHeadClient.cancelAllGoals();
        state = FocusHead::IDLE;
    }

    bool goalCallback(FocusHeadActionServer::GoalHandle gh) {
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Executing goal callback for FocusHeadAction");

        if (state == FocusHead::LOOKING_AT_PATH && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_INFO("Rejecting dog target because currently looking at path");
            gh.setRejected();
            return false;
        }
        if (state == FocusHead::LOOKING_FOR_DOG && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_INFO("Ignoring additional request to look for dog");
            gh.setRejected();
            return false;
        }

        if (state != FocusHead::IDLE) {
            ROS_INFO("Pre-empting current focus head target for new goal");
            currentGH.setCanceled();
            pointHeadClient.cancelGoal();
        }

        currentGH = gh;
        currentGH.setAccepted();
        if(!execute(currentGH.getGoal())){
            ROS_ERROR("Failed to schedule point head action");
            currentGH.setAborted();
            state = FocusHead::IDLE;
            return false;
        }
        return true;
    }

    void pointHeadCompleteCallback(const actionlib::SimpleClientGoalState& goalState,
            const pr2_controllers_msgs::PointHeadResultConstPtr result){
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Received point head complete callback");
        currentGH.setSucceeded();
        timeout.stop();
        state = FocusHead::IDLE;
    }

    bool execute(const FocusHeadActionServer::GoalConstPtr goal) {
        ROS_INFO("Inside execute for focusHead");

        pr2_controllers_msgs::PointHeadGoal phGoal;
        if (goal->target == FocusHeadGoal::DOG_TARGET) {
            state = FocusHead::LOOKING_FOR_DOG;
            // If the last position is known, use that as our start point.
            if (goal->isPositionSet) {
                phGoal.target = goal->position;
            }
            else {
                // Use the current right hand position at height 0 as the starting point.
                // Determine the position of the hand in the base frame.
                PointStamped handInBaseFrame;
                {
                    PointStamped handInHandFrame;
                    handInHandFrame.header.frame_id = "r_wrist_roll_link";
                    try {
                        tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame,
                                handInHandFrame.header.frame_id, handInBaseFrame);
                    }
                    catch (tf::TransformException& ex) {
                        ROS_ERROR(
                                "Failed to transform hand position to /base_footprint");
                        return false;
                    }
                }
                handInBaseFrame.header.frame_id = "/base_footprint";
                handInBaseFrame.header.stamp = ros::Time::now();
                handInBaseFrame.point.z = 0;
                phGoal.target = handInBaseFrame;
            }
        }
        else if (goal->target == FocusHeadGoal::PATH_TARGET) {
            state = FocusHead::LOOKING_AT_PATH;
            phGoal.target = goal->position;
            // rosrun tf tf_echo base_link wide_stereo_link
            phGoal.target.point.z = 1.25;
        }
        else {
            ROS_ERROR("Unknown focus target type: %s", goal->target.c_str());
            return false;
        }

        pointHeadClient.sendGoal(phGoal,
                boost::bind(&FocusHead::pointHeadCompleteCallback, this, _1, _2));
        timeout.start();

        return true;
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
    ros::Timer timeout;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "focus_head_action");
    FocusHead action(ros::this_node::getName());
    ros::spin();

    return 0;
}
