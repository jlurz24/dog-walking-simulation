#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/constants/constants.hpp>
// Generated messages
#include <dogsim/AdjustDogPositionAction.h>
#include <dogsim/MoveRobotAction.h>

namespace {
  using namespace std;

  typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
  typedef actionlib::SimpleActionClient<dogsim::MoveRobotAction> MoveRobotClient;

  class AdjustDogPositionAction {
    public:
      AdjustDogPositionAction(const string& name): as(nh, name, boost::bind(&AdjustDogPositionAction::adjust, this, _1), false), actionName(name),
        rightArm("move_right_arm", true),
        moveRobot("move_robot_action", true){
    
        rightArm.waitForServer(ros::Duration(5.0));
        moveRobot.waitForServer(ros::Duration(5.0));
        
        as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
        
        nh.param("leash_length", leashLength, 2.0);
      
        startPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/start_viz", 1);

        ROS_INFO("Ending init of the adjust dog position action");
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
    if(moveRobot.getState() == actionlib::SimpleClientGoalState::ACTIVE){
        moveRobot.cancelGoal();
    }
    as.setPreempted();
  }

  void adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    ROS_DEBUG("Adjusting dog position");
      
    if(!as.isActive()){
        ROS_INFO("Adjust dog position action cancelled prior to start");
        return;
    }

    geometry_msgs::PointStamped goalInBaseFrame;
    try {
        tf.transformPoint("/r_wrist_roll_link", ros::Time(0), goal->goalPosition, goal->goalPosition.header.frame_id, goalInBaseFrame);
    }
    catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform goal point to r_wrist_roll_link");
        as.setAborted();
        return;
    }
    ROS_INFO("Goal in hand frame x: %f y: %f z: %f", goalInBaseFrame.point.x, goalInBaseFrame.point.y, goalInBaseFrame.point.z);

    geometry_msgs::PoseStamped dogInBaseFrame;
    try {
        tf.transformPose("/r_wrist_roll_link", ros::Time(0), goal->dogPose, goal->dogPose.header.frame_id, dogInBaseFrame);
    }
    catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform dog pose to r_wrist_roll_link");
        as.setAborted();
        return;
    }
    ROS_INFO("Dog position in hand frame x: %f y: %f z: %f", dogInBaseFrame.pose.position.x, dogInBaseFrame.pose.position.y, dogInBaseFrame.pose.position.z);
    assert(goalInBaseFrame.header.frame_id == dogInBaseFrame.header.frame_id);
    
    
    // Determine the position of the base in the hand frame
    geometry_msgs::PointStamped handInBaseFrame;
    {
        geometry_msgs::PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "/r_wrist_roll_link";
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame, handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform base position to r_wrist_roll_link");
            as.setAborted();
            return;
        }
        ROS_INFO("Hand position in base frame x: %f y: %f z: %f", handInBaseFrame.point.x, handInBaseFrame.point.y, handInBaseFrame.point.z);
    }
    
    // Givens: Dog position + goal position
    // create a line between the dog position and goal position
    // select a point on that line that is leash distance from the dog towards
    // goal.
    // TODO: Allow other points on the arc to be acceptable.
    // TODO: Select random acceptable points on z axis.
    
    // Determine the horizontal position assuming the desired height of the arm is 1.
    double armHeight = -dogInBaseFrame.pose.position.z; // Height of the robot hand relative to the dog.
    double planarLeashLength = sqrt(utils::square(leashLength) - utils::square(armHeight));
    ROS_DEBUG("Arm height: %f planar leash length: %f", armHeight, planarLeashLength);

    // Find the angle between the dog and the goal point.
    // Calculate the unit vector given x1, y1 = dog and x2, y2 = goal
    double ux = goalInBaseFrame.point.x - dogInBaseFrame.pose.position.x;
    double uy = goalInBaseFrame.point.y - dogInBaseFrame.pose.position.y;
    double length = sqrt(utils::square(ux) + utils::square(uy));
    if(length > numeric_limits<double>::min()){
        ux /= length;
        uy /= length;
    }
    
    // Now update the goal to move to the dog to the goal point.
    geometry_msgs::PointStamped start;
    start.point.x = goalInBaseFrame.point.x + planarLeashLength * ux;
    start.point.y = goalInBaseFrame.point.x + planarLeashLength * uy;
    start.point.z = armHeight;
    start.header = dogInBaseFrame.header;
    
    ROS_INFO("Start position in hand frame x: %f y: %f z: %f", start.point.x, start.point.y, start.point.z);   
    
    // Convert to the base frame.
    geometry_msgs::PointStamped startInBaseFrame;
    try {
        tf.transformPoint("/base_footprint", ros::Time(0), start, start.header.frame_id, startInBaseFrame);
    }
    catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform dog pose to r_wrist_roll_link");
        as.setAborted();
        return;
    }
    
    if(startPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA ORANGE = utils::createColor(1, 0.5, 0);
      visualization_msgs::Marker startMsg = utils::createMarker(startInBaseFrame.point, startInBaseFrame.header, ORANGE, false);
      startPub.publish(startMsg);
    }
    // The base to the start position
    geometry_msgs::PointStamped baseGoal;
    baseGoal.header = startInBaseFrame.header;
    baseGoal.point.z = 0;
    baseGoal.point.x = startInBaseFrame.point.x - handInBaseFrame.point.x;
    baseGoal.point.y = startInBaseFrame.point.y - handInBaseFrame.point.y;
      
    // Check if we are still active
    if(!as.isActive()){
        return;
    }
    
    dogsim::MoveRobotGoal moveGoal;
    moveGoal.position = baseGoal;
    moveRobot.sendGoal(moveGoal);
    
    // The caller should abort the movement if it takes too long.
    moveRightArm(start);
    moveRobot.waitForResult(ros::Duration(1.0));
    if(moveRobot.getState() == actionlib::SimpleClientGoalState::ACTIVE){
        moveRobot.cancelGoal();
    }
    if(rightArm.getState() == actionlib::SimpleClientGoalState::ACTIVE){
        rightArm.cancelGoal();
    }
    as.setSucceeded();
  }

  bool moveRightArm(const geometry_msgs::PointStamped goalPoint){
     ROS_DEBUG("Moving arm to position %f %f %f in frame %s", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str());

     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 1;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(0.25);
     goal.motion_plan_request.expected_path_duration = ros::Duration(1.0);
     goal.motion_plan_request.expected_path_dt = ros::Duration(0.25);
     arm_navigation_msgs::SimplePoseConstraint desiredPos;
     desiredPos.header.frame_id = goalPoint.header.frame_id;
     desiredPos.header.stamp = ros::Time::now();
     desiredPos.link_name = "r_wrist_roll_link";
     desiredPos.pose.position = goalPoint.point;
     desiredPos.absolute_position_tolerance.x = 0.04;
     desiredPos.absolute_position_tolerance.y = 0.04;
     desiredPos.absolute_position_tolerance.z = 0.04;
    
     desiredPos.pose.orientation.x = desiredPos.pose.orientation.y = desiredPos.pose.orientation.z = 0;
     desiredPos.pose.orientation.w = 1;
     // Allow any wrist position. We don't care about knots right now.
     desiredPos.absolute_roll_tolerance = desiredPos.absolute_pitch_tolerance = desiredPos.absolute_yaw_tolerance = 2 * boost::math::constants::pi<double>();

     arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desiredPos, goal);

     goal.disable_collision_monitoring = true;
     goal.accept_invalid_goals = true;
     goal.accept_partial_plans = true;
     rightArm.sendGoal(goal);
     return true;
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
    string actionName;

    MoveArmClient rightArm;
    MoveRobotClient moveRobot;
    
    tf::TransformListener tf;

    //! Publisher for start position.
    ros::Publisher startPub;
  
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
