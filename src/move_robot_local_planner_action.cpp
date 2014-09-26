#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <boost/thread.hpp>

// Generated messages
#include <dogsim/MoveRobotAction.h>

namespace {
using namespace std;

class MoveRobotLocalPlannerAction {
public:
    MoveRobotLocalPlannerAction(const string& name) :
            as(nh, name, boost::bind(&MoveRobotLocalPlannerAction::move, this, _1), false), actionName(
                    name), tf(ros::Duration(10)), costmap("local_costmap", tf) {
        as.registerPreemptCallback(boost::bind(&MoveRobotLocalPlannerAction::preemptCB, this));

        // Set up the publisher for the cmd_vel topic
        cmdVelocityPub = nh.advertise<geometry_msgs::Twist>("base_controller/command", 1, true);
        costmap.start();
        tp.initialize("local_planner", &tf, &costmap);
        as.start();

        // Initialize the command thread
        baseCommandThread = unique_ptr<boost::thread>(new boost::thread(boost::bind(&MoveRobotLocalPlannerAction::sendCommandLoop, this)));
    }

    ~MoveRobotLocalPlannerAction(){
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

        {
            boost::mutex::scoped_lock(cmdLock);
            baseCmd.linear.x = baseCmd.linear.y = baseCmd.linear.z = 0.0;
            baseCmd.angular.z = 0.0;
        }
        // This will be published by the command loop
    }

    bool move(const dogsim::MoveRobotGoalConstPtr& goal) {
        // TODO: Make this activate, which just starts listening for moves
        ROS_DEBUG("Received Move Robot Goal with: %lu poses", goal->poses.size());

        if (!as.isActive()) {
            ROS_INFO("Move robot action canceled prior to start");
            return false;
        }

        assert(tp.isInitialized());

        // TODO: Move all this to the planner
        if (goal->poses.size() == 0) {
            ROS_ERROR("No goal poses sent to local planner");
            as.setAborted();
            return false;
        }

        if (goal->poses[0].header.frame_id != costmap.getGlobalFrameID()) {
            ROS_ERROR("Goal frame %s must match costmap frame %s",
                    goal->poses[0].header.frame_id.c_str(), costmap.getGlobalFrameID().c_str());
            as.setAborted();
            return false;
        }

        tf::Stamped<tf::Pose> globalPose;
        if (!costmap.getRobotPose(globalPose)) {
            ROS_ERROR(
                    "Cannot make a plan because the local planner could not get the start pose of the robot");
            as.setAborted();
            return false;
        }

        // Loop over the path and select the next target at each callback
        ros::Duration pathUpdateRate(2, 0);

        // How far to look ahead.
        ros::Duration pathLookahead(2, 0);

        // Loop and move the robot, checking the position of the goal in the base
        // frame after each move.
        ros::Rate updateRate(10);

        bool success = false;
        while (ros::ok() && as.isActive() && !as.isPreemptRequested()) {
            const ros::Time nextUpdateTime = ros::Time::now() + pathUpdateRate;

            ROS_DEBUG("Recomputing path segment");
            vector<geometry_msgs::PoseStamped>::const_iterator i;

            // We always want to include the last pose if there are no
            // future poses.
            for (i = goal->poses.begin(); i != goal->poses.end() - 1; ++i) {
                if (i->header.stamp > ros::Time::now()) {
                    break;
                }
            }

            vector<geometry_msgs::PoseStamped>::const_iterator j;
            for (j = i; j != goal->poses.end(); ++j) {
                if (j->header.stamp > ros::Time::now() + pathLookahead) {
                    break;
                }
            }

            // Create a list of all future poses.
            const vector<geometry_msgs::PoseStamped> currentPath = vector<geometry_msgs::PoseStamped>(i, j);
            assert(currentPath.size() > 0 && "No poses in current path");

            ROS_DEBUG("Setting plan for local planner");
            if (!tp.setPlan(currentPath)) {
                ROS_ERROR("Failed to set plan");
                as.setAborted();
                return false;
            }

            while (ros::ok() && as.isActive() && !as.isPreemptRequested() && !tp.isGoalReached() && ros::Time::now() < nextUpdateTime) {

                geometry_msgs::Twist baseCmdTemp;
                if (!tp.computeVelocityCommands(baseCmdTemp)) {
                    ROS_WARN("Failed to compute velocity commands. Publishing zero velocity.");
                    {
                        boost::mutex::scoped_lock(cmdLock);
                        baseCmd.linear.x = 0.0;
                        baseCmd.linear.y = 0.0;
                        baseCmd.angular.z = 0.0;
                    }

                }
                else {
                    ROS_DEBUG("Successfully computed base command %f %f %f", baseCmd.linear.x,
                            baseCmd.linear.y, baseCmd.angular.z);
                    {
                        boost::mutex::scoped_lock(cmdLock);
                        baseCmd = baseCmdTemp;
                    }
                }
                updateRate.sleep();
            }

            if(tp.isGoalReached()){
                ROS_DEBUG("Local planner reached goal");
            }
            if(ros::Time::now() >= nextUpdateTime){
                ROS_DEBUG("Exiting loop because next update time reached");
            }
            if(!ros::ok() && !as.isActive() && as.isPreemptRequested()){
                ROS_INFO("Exiting due to premption or failure");
            }

            if(i == goal->poses.end() - 1 && tp.isGoalReached()){
                success = true;
                ROS_DEBUG("Path completed successfully");
                break;
            }
        }

        // Stop movement
        ROS_INFO("Stopping movement as movement plan is complete");
        {
            boost::mutex::scoped_lock(cmdLock);
            baseCmd.linear.x = 0.0;
            baseCmd.linear.y = 0.0;
            baseCmd.angular.z = 0.0;
        }

        if(success){
            as.setSucceeded();
        }
        else {
            as.setAborted();
        }
        return success;
    }

private:

    void sendCommandLoop(){
        static const boost::posix_time::seconds CYCLE_TIME = boost::posix_time::seconds(1.0 / 50.0);
        while(true){
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

    /*
     * Costmap for the planner
     */
    costmap_2d::Costmap2DROS costmap;

    /*
     * Local planner
     */
    dwa_local_planner::DWAPlannerROS tp;

    //! Publisher for command velocities
    ros::Publisher cmdVelocityPub;

    //! Thread that executes the base movement loop
    unique_ptr<boost::thread> baseCommandThread;

    //! Current command. This will be sent until it is updated again.
    geometry_msgs::Twist baseCmd;

    //! Lock for the current command
    boost::mutex cmdLock;
}
;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_robot_action");
    MoveRobotLocalPlannerAction action(ros::this_node::getName());
    ros::spin();
    return 0;
}
