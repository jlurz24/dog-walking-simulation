#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <dogsim/utils.h>

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
      nh.param("leash-length", leashLength, 2.0);
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

    // Givens: Dog position + goal position
    // create a line between the dog position and goal position
    // select a point on that line that is leash distance from the dog towards
    // goal
    // create a path for the arm including the two points
    // TODO: Make leash length a param
    // TODO: Should we start by finding a legal position spot? We could
    //       start that work immediately while we find a path from there to
    //       the final position.
    // TODO: Select random acceptable points on z axis.
    
    // Find the angle between the dog and the goal point.
    geometry_msgs::PointStamped start;

    // Determine the horizontal position assuming the desired height of the arm is 1.
    double armHeight = 1; // TODO: Make this dynamic.
    double planarLeashLength = sqrt(utils::square(leashLength) - utils::square(armHeight));
    double dgA = atan2(goal->goalPosition.point.y - goal->dogPose.pose.position.y, goal->goalPosition.point.x - goal->dogPose.pose.position.x);
    start.point.x = goal->dogPose.pose.position.x + planarLeashLength * sin(dgA);
    start.point.y = goal->dogPose.pose.position.y + planarLeashLength * cos(dgA);
    start.point.z = armHeight;
    start.header.frame_id = goal->dogPose.header.frame_id;
    moveRightArm(start);

    // TODO: Need to wait for the result and then move the arm to correct the dog position.
    // Don't wait for the result.
    as.setSucceeded(result);
  }

  void moveRightArm(const geometry_msgs::PointStamped goalPoint){
     ROS_INFO("Moving to position %f %f %f in frame %s", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str());
     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 3;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

     arm_navigation_msgs::PositionConstraint desiredPos;
     desiredPos.header.frame_id = goalPoint.header.frame_id;
     desiredPos.link_name = "r_wrist_roll_link";
     desiredPos.position = goalPoint.point;
     desiredPos.constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;

     // TODO: More intelligently define the goal region.
     desiredPos.constraint_region_shape.dimensions.push_back(0.25);
     desiredPos.constraint_region_shape.dimensions.push_back(0.25);
     desiredPos.constraint_region_shape.dimensions.push_back(0.25);
     desiredPos.weight = 1;

     goal.disable_collision_monitoring = true;
     goal.motion_plan_request.goal_constraints.position_constraints.push_back(desiredPos);

     rightArm.sendGoal(goal);
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
