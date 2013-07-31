#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
// Generated messages
#include <dogsim/AdjustDogPositionAction.h>

namespace {
  using namespace std;

  typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;

  class AdjustDogPositionAction {
    public:
      AdjustDogPositionAction(const string& name): as(nh, name, boost::bind(&AdjustDogPositionAction::adjust, this, _1), false), actionName(name), rightArm("move_right_arm", true) {
    
      rightArm.waitForServer(ros::Duration(5.0));
  
      as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
      nh.param("leash_length", leashLength, 2.0);
      ROS_INFO("Starting init of the adjust dog position action");
      as.start();
  }
  
  void preemptCB(){
    ROS_INFO("Preempting the adjust dog position action");

    if(!as.isActive()){
      ROS_INFO("Adjust dog position action cancelled prior to start");
      return;
    }

    if(rightArm.getState() == actionlib::SimpleClientGoalState::ACTIVE){
      rightArm.cancelGoal();
    }
    as.setPreempted();
  }

  void adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Adjust dog position action cancelled prior to start");
      return;
    }

    geometry_msgs::PointStamped goalInBaseFrame;
    try {
      tf.transformPoint("/base_footprint", ros::Time(0), goal->goalPosition, goal->goalPosition.header.frame_id, goalInBaseFrame);
    }
    catch(tf::TransformException& ex){
      ROS_INFO("Failed to transform point to base footprint");
      return;
    }

    // Givens: Dog position + goal position
    // create a line between the dog position and goal position
    // select a point on that line that is leash distance from the dog towards
    // goal
    // create a path for the arm including the two points
    // TODO: Should we start by finding a legal position spot? We could
    //       start that work immediately while we find a path from there to
    //       the final position.
    // TODO: Select random acceptable points on z axis.
    
    // Find the angle between the dog and the goal point.
    geometry_msgs::PointStamped start;

    // Determine the horizontal position assuming the desired height of the arm is 1.
    double armHeight = 0.916; // Initial height of the robot hand
    double planarLeashLength = sqrt(utils::square(leashLength) - utils::square(armHeight));
    double dgA = atan2(goalInBaseFrame.point.y - goal->dogPose.pose.position.y, goalInBaseFrame.point.x - goal->dogPose.pose.position.x);
    start.point.x = goal->dogPose.pose.position.x + planarLeashLength * sin(dgA);
    start.point.y = goal->dogPose.pose.position.y + planarLeashLength * cos(dgA);
    start.point.z = armHeight;
    start.header.frame_id = goal->dogPose.header.frame_id;
    ROS_INFO("Moving arm to start position for correction");
    bool success = moveRightArm(start);
    if(!success){
      ROS_INFO("Failed to move arm to start position");
      as.setAborted();
      return;
    }

    // Now update the goal to move to the dog to the goal point.
    double distance = utils::pointToPointXYDistance(goal->dogPose.pose.position, goalInBaseFrame.point);
    geometry_msgs::PointStamped end;
    end.header.frame_id = goal->dogPose.header.frame_id;
    end.point.x = start.point.x + distance * sin(dgA);
    end.point.y = start.point.y + distance * cos(dgA);
    end.point.z = armHeight;
    ROS_INFO("Moving arm %f distance to correct dog", distance);
    success = moveRightArm(end);
    if(!success){
      ROS_INFO("Failed to move arm to correct dog from start position");
      as.setAborted();
      return;
    }
    ROS_INFO("Dog correction completed");
    as.setSucceeded(result);
  }

  bool moveRightArm(const geometry_msgs::PointStamped goalPoint){
     ROS_INFO("Moving to position %f %f %f in frame %s", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str());

     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 1;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

     arm_navigation_msgs::SimplePoseConstraint desiredPos;
     desiredPos.header.frame_id = goalPoint.header.frame_id;
     desiredPos.header.stamp = ros::Time::now();
     desiredPos.link_name = "r_wrist_roll_link";
     desiredPos.pose.position = goalPoint.point;
     desiredPos.absolute_position_tolerance.x = 0.02;
     desiredPos.absolute_position_tolerance.y = 0.02;
     desiredPos.absolute_position_tolerance.z = 0.02;
    
     desiredPos.pose.orientation.x = desiredPos.pose.orientation.y = desiredPos.pose.orientation.z = 0;
     desiredPos.pose.orientation.w = 1;
     desiredPos.absolute_roll_tolerance = desiredPos.absolute_pitch_tolerance = desiredPos.absolute_yaw_tolerance = 1;

     arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desiredPos, goal);

     goal.disable_collision_monitoring = true;
     return utils::sendGoal(&rightArm, goal, nh, 5.0 /** Timeout **/);
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
    string actionName;

    // create messages that are used to published feedback/result
    dogsim::AdjustDogPositionFeedback feedback;
    dogsim::AdjustDogPositionResult result;

    MoveArmClient rightArm;
  
    tf::TransformListener tf;

    double leashLength;
};
}

int main(int argc, char** argv){
  ROS_INFO("Main function for adjust_dog_position_action");
  ros::init(argc, argv, "adjust_dog_position_action");
  AdjustDogPositionAction action(ros::this_node::getName());
  ROS_INFO("Waiting for actions");
  ros::spin();
  
  return 0;
}
