#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <boost/math/constants/constants.hpp>

// Generated messages
#include <dogsim/MoveArmToBasePositionAction.h>

namespace {
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
    using namespace std;
    using namespace ros;

    class MoveArmToBasePosition {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::MoveArmToBasePositionAction> as;
            string actionName;
            TrajClient trajectoryClient;

        public:
            MoveArmToBasePosition(const string& name):as(nh, name, boost::bind(&MoveArmToBasePosition::moveToBasePosition, this), false),
                                    actionName(name),
                                    trajectoryClient("r_arm_controller/joint_trajectory_action"){
            trajectoryClient.waitForServer();
            as.start();
        }

        void moveToBasePosition(){
            if(!as.isActive()){
                ROS_INFO("Move arm to base position action cancelled prior to start");
                return;
            }
            
            ROS_INFO("Moving arm to base position");
            // When to start the trajectory: now
            pr2_controllers_msgs::JointTrajectoryGoal goal = armBaseTrajectory();
            goal.trajectory.header.stamp = ros::Time::now();
            utils::sendGoal(&trajectoryClient, goal, nh, 2.0);
            as.setSucceeded();
        }

        pr2_controllers_msgs::JointTrajectoryGoal armBaseTrajectory(){
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
            goal.trajectory.points[0].positions[0] = -boost::math::constants::pi<double>() / 4.0;
            goal.trajectory.points[0].positions[1] = 0.4;
            goal.trajectory.points[0].positions[2] = -boost::math::constants::pi<double>() / 4.0;
            goal.trajectory.points[0].positions[3] = -boost::math::constants::pi<double>() / 4.0;
            goal.trajectory.points[0].positions[4] = -1.0;
            goal.trajectory.points[0].positions[5] = -0.2;
            goal.trajectory.points[0].positions[6] = 0.0;
   
            // Velocities
            goal.trajectory.points[0].velocities.resize(7);
    
            // To be reached 2 seconds after starting along the trajectory
            goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
            return goal;
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_arm_to_base_position_action");
  MoveArmToBasePosition action(ros::this_node::getName());

  ros::spin();
}

