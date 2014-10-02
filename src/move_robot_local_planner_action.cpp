#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>
#include <message_filters/subscriber.h>

// Generated messages
#include <dogsim/MoveRobotAction.h>
#include <dogsim/NextGoal.h>

namespace {
using namespace std;

class MoveRobotLocalPlannerAction {
public:
    MoveRobotLocalPlannerAction(const string& name) :
        as(nh, name, boost::bind(&MoveRobotLocalPlannerAction::activate, this, _1), false), actionName(
                name) {
        as.registerPreemptCallback(boost::bind(&MoveRobotLocalPlannerAction::preemptCB, this));

        // Set up the publisher for the cmd_vel topic
        cmdVelocityPub = nh.advertise<geometry_msgs::Twist>("base_controller/command", 1, true);

        nextGoalSub.reset(
                new message_filters::Subscriber<dogsim::NextGoal>(nh, "/robot_path/next_goal", 1));
        nextGoalSub->registerCallback(boost::bind(&MoveRobotLocalPlannerAction::updateGoal, this, _1));
        nextGoalSub->unsubscribe();

        as.start();

        // Initialize the command thread
        baseCommandThread = unique_ptr < boost::thread
                > (new boost::thread(
                        boost::bind(&MoveRobotLocalPlannerAction::sendCommandLoop, this)));
    }

    ~MoveRobotLocalPlannerAction() {
        baseCommandThread->interrupt();
    }

protected:
    void preemptCB() {
        ROS_DEBUG("Preempting the move robot action");

        if (!as.isActive()) {
            ROS_INFO("Move robot position action canceled prior to start");
            return;
        }

        // Do not stop the robot as this will be called if we are preempted by the sequencer.
        as.setPreempted();
    }

    void stop() {
        ROS_INFO("Stopping the base");
        // This will be published by the command loop
        {
            boost::mutex::scoped_lock(cmdLock);
            baseCmd.linear.x = baseCmd.linear.y = baseCmd.linear.z = 0.0;
            baseCmd.angular.z = 0.0;
        }

        nextGoalSub->unsubscribe();
    }

    bool activate(const dogsim::MoveRobotGoalConstPtr& goal) {

        if (!as.isActive()) {
            ROS_INFO("Move robot action canceled prior to start");
            return false;
        }

        nextGoalSub->subscribe();

        as.setSucceeded();
        return true;
    }

private:

    void updateGoal(const dogsim::NextGoalConstPtr& goal) {
        {
            boost::mutex::scoped_lock(cmdLock);
            baseCmd = goal->twist;
        }
    }

    void sendCommandLoop() {
        static const boost::posix_time::seconds CYCLE_TIME = boost::posix_time::seconds(1.0 / 25.0);
        while (true) {
            // Publish the command to the base
            geometry_msgs::Twist baseCmdTemp;
            {
                boost::mutex::scoped_lock(cmdLock);
                baseCmdTemp = baseCmd;
            }
            cmdVelocityPub.publish(baseCmdTemp);
        }
        boost::this_thread::sleep(CYCLE_TIME);
    }

    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::MoveRobotAction> as;
    string actionName;

    //! We will be listening to TF transforms
    tf::TransformListener tf;

    //! Publisher for command velocities
    ros::Publisher cmdVelocityPub;

    //! Thread that executes the base movement loop
    unique_ptr<boost::thread> baseCommandThread;

    //! Current command. This will be sent until it is updated again.
    geometry_msgs::Twist baseCmd;

    //! Lock for the current command
    boost::mutex cmdLock;

    //! Current goal subscriber
    unique_ptr<message_filters::Subscriber<dogsim::NextGoal> > nextGoalSub;
}
;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_robot_action");
    MoveRobotLocalPlannerAction action(ros::this_node::getName());
    ros::spin();
    return 0;
}
