#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>

// Generated messages
#include <dogsim/MoveDogAwayAction.h>

namespace {
    typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
    using namespace std;
    using namespace ros;

    class MoveDogAway {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::MoveDogAwayAction> as;
            string actionName;
            TrajClient trajectoryClient;

        public:
            MoveDogAway(const string& name):as(nh, name, boost::bind(&MoveDogAway::moveAway, this), false),
                                    actionName(name),
                                    trajectoryClient("r_arm_controller/joint_trajectory_action"){
            trajectoryClient.waitForServer();
            as.start();
        }

        void moveAway(){
            if(!as.isActive()){
                ROS_INFO("Move dog away action cancelled prior to start");
                return;
            }
            
            ROS_INFO("Moving dog out of the way");
            // When to start the trajectory: now
            pr2_controllers_msgs::JointTrajectoryGoal goal = armExtensionTrajectory();
            goal.trajectory.header.stamp = ros::Time::now();
            utils::sendGoal(&trajectoryClient, goal, nh, 2.0);
            as.setSucceeded();
        }

        pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory(){
            pr2_controllers_msgs::JointTrajectoryGoal goal;
    
            // Setup joint names
            goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

            // We will have two waypoints in this goal trajectory
            goal.trajectory.points.resize(1);

            // Positions
            goal.trajectory.points[0].positions.resize(7);
            goal.trajectory.points[0].positions[0] = -2.0;
            goal.trajectory.points[0].positions[3] = -1.2;
            goal.trajectory.points[0].positions[5] = 0.0;
   
            // Velocities
            goal.trajectory.points[0].velocities.resize(7);
    
            // To be reached 2 seconds after starting along the trajectory
            goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
            return goal;
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_dog_away");
  MoveDogAway action(ros::this_node::getName());
  ros::spin();
}

