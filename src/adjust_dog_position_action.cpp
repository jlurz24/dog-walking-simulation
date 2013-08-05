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
  
        as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
        nh.param("leash_length", leashLength, 2.0);
      
        startPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/start_viz", 1);
        goalPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/goal_viz", 1);
    
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
    
    assert(goalInBaseFrame.header.frame_id == goal->dogPose.header.frame_id);
    
    // Calculate the unit vector given x1, y1 = dog and x2, y2 = goal
    double ux = goalInBaseFrame.point.x - goal->dogPose.pose.position.x;
    double uy = goalInBaseFrame.point.y - goal->dogPose.pose.position.y;
    double length = sqrt(utils::square(ux) + utils::square(uy));
    if(length > numeric_limits<double>::min()){
        ux /= length;
        uy /= length;
    }
    
    start.point.x = goal->dogPose.pose.position.x + planarLeashLength * ux;
    start.point.y = goal->dogPose.pose.position.y + planarLeashLength * uy;
    start.point.z = armHeight;
    
    start.header.frame_id = goal->dogPose.header.frame_id;
    ROS_INFO("Moving arm to start position for correction");
    
    if(startPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA ORANGE = utils::createColor(1, 0.5, 0);
      visualization_msgs::Marker startMsg = utils::createMarker(start.point, start.header, ORANGE, true);
      startMsg.lifetime = ros::Duration(2.5);
      startPub.publish(startMsg);
    }
    
    bool success = moveRightArm(start);
    if(!success){
      ROS_INFO("Failed to move arm to start position before base movement");
      
      // Start moving the base to that position and then retry.
      // Select a point along the line between the robot and the goal
      const double XY_OFFSET_FROM_START = 0.25;
      double baseDistanceForArm = sqrt(utils::square(XY_OFFSET_FROM_START) + utils::square(XY_OFFSET_FROM_START));
      
      // Robot position is 0,0 because movement is in the base frame.
      // Calculate the unit vector given:
      // x1, y1 = start position
      // x2, y2 = robot position.
      double sLength = sqrt(utils::square(0 - start.point.x) + utils::square(0 - start.point.y));
      
      double sx = 0 - start.point.x;
      double sy = 0 - start.point.y;
      if(sLength > numeric_limits<double>::min()){
        sx /= sLength;
        sy /= sLength;
      }
      
      geometry_msgs::PointStamped baseGoal;
      baseGoal.header = start.header;
      baseGoal.point.z = 0;
      baseGoal.point.x = start.point.x + baseDistanceForArm * sx;
      baseGoal.point.y = start.point.y + baseDistanceForArm * sy;
      
      // The caller will abort the movement if it takes too long.
      dogsim::MoveRobotGoal moveGoal;
      moveGoal.position = baseGoal;
      success = utils::sendGoal(&moveRobot, moveGoal, nh, 5.0 /** Timeout **/);
      
      // Try to move the arm again if we were successful in moving the base.
      if(success){
          ROS_INFO("Attempting to move the arm after base movement");
          success = moveRightArm(start);
      }
      
      if(!success){
        ROS_INFO("Failed to move arm to start position after base movement");
        as.setAborted();
        return;
      }
    }

    // Now update the goal to move to the dog to the goal point.
    double distance = utils::pointToPointXYDistance(goal->dogPose.pose.position, goalInBaseFrame.point);
    geometry_msgs::PointStamped end;
    end.header.frame_id = goal->dogPose.header.frame_id;
    end.point.x = start.point.x + distance * ux;
    end.point.y = start.point.y + distance * uy;
    end.point.z = armHeight;
    ROS_INFO("Moving arm %f distance to correct dog", distance);
    
    if(goalPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
      visualization_msgs::Marker endMsg = utils::createMarker(end.point, end.header, GREEN, true);
      endMsg.lifetime = ros::Duration(2.5);
      goalPub.publish(endMsg);
    }
    
    success = moveRightArm(end);
    if(!success){
      ROS_INFO("Failed to move arm to correct dog from start position");
      as.setAborted();
      return;
    }
    ROS_INFO("Dog correction completed");
    as.setSucceeded();
  }

  bool moveRightArm(const geometry_msgs::PointStamped goalPoint){
     ROS_DEBUG("Moving arm to position %f %f %f in frame %s", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str());

     arm_navigation_msgs::MoveArmGoal goal;
     goal.motion_plan_request.group_name = "right_arm";
     goal.motion_plan_request.num_planning_attempts = 1;
     goal.motion_plan_request.planner_id = "";
     goal.planner_service_name = "ompl_planning/plan_kinematic_path";
     goal.motion_plan_request.allowed_planning_time = ros::Duration(1.0);

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
     return utils::sendGoal(&rightArm, goal, nh, 5.0 /** Timeout **/);
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
    
    //! Publisher for goal position.
    ros::Publisher goalPub;
  
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
