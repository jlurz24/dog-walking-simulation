#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>

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
        if(state == FocusHead::LOOKING_FOR_DOG){
            dogPositionSub->unsubscribe();
        }
        currentGH.setCanceled();
        pointHeadClient.cancelGoal();
    }

    void timeoutCallback(const ros::TimerEvent& e){
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_DEBUG("Received timeout callback");

        // Ignore any timeouts after completion
        if(state == FocusHead::IDLE){
            return;
        }

        if(state == FocusHead::LOOKING_FOR_DOG){
            dogPositionSub->unsubscribe();
        }
        currentGH.setAborted();
        pointHeadClient.cancelAllGoals();
        state = FocusHead::IDLE;
    }

    bool goalCallback(FocusHeadActionServer::GoalHandle gh) {
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_DEBUG("Executing goal callback for FocusHeadAction");

        if (state == FocusHead::LOOKING_AT_PATH && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Rejecting dog target because currently looking at path");
            gh.setRejected();
            return false;
        }
        if (state == FocusHead::LOOKING_FOR_DOG && gh.getGoal()->target == FocusHeadGoal::DOG_TARGET) {
            ROS_DEBUG("Ignoring additional request to look for dog");
            gh.setRejected();
            return false;
        }

        if (state != FocusHead::IDLE) {
            ROS_DEBUG("Prempting current focus head target for new goal");
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
        ROS_DEBUG("Received point head complete callback");
        currentGH.setSucceeded();
        timeout.stop();
        if(state == FocusHead::LOOKING_FOR_DOG){
            dogPositionSub->unsubscribe();
        }
        state = FocusHead::IDLE;
    }

    bool execute(const FocusHeadActionServer::GoalConstPtr goal) {
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
        timeout = nh.createTimer(ros::Duration(1.0), &FocusHead::timeoutCallback, this,
                true /* One shot */);
        dogPositionSub->subscribe();
        return true;
    }

    void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
        if(dogPosition->unknown || state != FocusHead::LOOKING_FOR_DOG){
            return;
        }
        boost::mutex::scoped_lock lock(stateMutex);
        ROS_INFO("Search for dog located the dog");

        currentGH.setSucceeded();
        timeout.stop();
        state = FocusHead::IDLE;
        dogPositionSub->unsubscribe();
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
