#pragma once
#include <ros/ros.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <geometry_msgs/Vector3.h>

namespace {
  using namespace std;

  class ArmMoveCostEstimator {
    public:
      ArmMoveCostEstimator(const robot_model::RobotModelConstPtr& robotMode, const string& groupName)
        :solver(robotMode, groupName, createVector(0.0, 0.0, -9.81)){}
    
      double calculateTotalForce(const moveit::planning_interface::MoveGroup::Plan& plan) const {
        double totalForce = 0;
        // Calculate the force in the start state.
        vector<double> lastTorques;
        vector<geometry_msgs::Wrench> wrenches;
        // Temp code
        for(unsigned int i = 0; i < plan.start_state_.joint_state.name.size(); ++i){
          ROS_INFO("Joint Name: %s", plan.start_state_.joint_state.name[i].c_str());
          ROS_INFO("Effort: %f", plan.start_state_.joint_state.effort[i]);
        }

        vector<double> accelerations(7);
        if(!solver.getTorques(plan.start_state_.joint_state.position, plan.start_state_.joint_state.velocity, accelerations, wrenches, lastTorques)){
          ROS_ERROR("Failed to solve for torques");
          return -1;
        }

        // Iterate over all the waypoints on the path.
        for(unsigned int i = 0; i < plan.trajectory_.getWayPointCount(); ++i){
            const robot_state::RobotState& waypoint = plan.trajectory_.getWayPoint(i);
            // Temp code
            for(unsigned int i = 0; i < plan.start_state_.joint_state.name.size(); ++i){
                ROS_INFO("Joint name: %s", waypoint.joint_state.name[i]);
                ROS_INFO("Effort: %f", waypoint.joint_state.effort[i]);
            }
        
            // Get the torques for the waypoint.
            vector<double> currentTorques;

            if(!solver.getTorques(waypoint.joint_state.position, waypoint.joint_state.velocity, accelerations, wrenches, currentTorques)){
                ROS_ERROR("Failed to solve for torques for waypoint %i", i);
                return -1;
            }

            // totalForce += calcuateForce(lastTorques, currentTorques, plan.trajectory_.getWaypointDurationFromPrevious(i));
            lastTorques = currentTorques;
        }
        return totalForce;
      }
    private:
      static geometry_msgs::Vector3 createVector(const double x, const double y, const double z){
          geometry_msgs::Vector3 result;
          result.x = x;
          result.y = y;
          result.z = z;
          return result;
      }
      dynamics_solver::DynamicsSolver solver;
    };
}

