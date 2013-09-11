#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/btVector3.h>

// Generated messages
#include <dogsim/MoveRobotAction.h>

namespace {
  using namespace std;

  static const double PI = boost::math::constants::pi<double>();
  
  class MoveRobotAction {
    public:
      MoveRobotAction(const string& name): as(nh, name, boost::bind(&MoveRobotAction::move, this, _1), false), actionName(name) {
        as.registerPreemptCallback(boost::bind(&MoveRobotAction::preemptCB, this));
      
        // Set up the publisher for the cmd_vel topic
        cmdVelocityPub = nh.advertise<geometry_msgs::Twist>("base_controller/command", 1, true);

        goalPub = nh.advertise<visualization_msgs::Marker>("move_robot_action/move_goal_viz", 1);
        movePub = nh.advertise<visualization_msgs::Marker>("move_robot_action/planned_move_viz", 1);
        as.start();
  }
  
  void preemptCB(){
    ROS_DEBUG("Preempting the move robot action");

    if(!as.isActive()){
      ROS_INFO("Move robot position action cancelled prior to start");
      return;
    }
    // Stop the robot.
    stop();
    as.setPreempted();
  }

  void stop(){
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    
    // Publish the command to the base
    cmdVelocityPub.publish(baseCmd);
  }
  
  void move(const dogsim::MoveRobotGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Move robot action cancelled prior to start");
      return;
    }
    
    // Visualize the goal.
    if(goalPub.getNumSubscribers()){
        std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
        goalPub.publish(utils::createMarker(goal->position.point, goal->position.header, GREEN, false));
    }
    
    // Determine the position in the map frame. This allows us to store
    // an absolute position
    geometry_msgs::PointStamped absoluteGoal;
    try {
        tf.transformPoint("/map", ros::Time(0), goal->position, goal->position.header.frame_id, absoluteGoal);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Failed to transform goal to map frame");
        as.setAborted();
        return;
    }
    
    // Determine the updated position of the robot.
    geometry_msgs::PointStamped goalPosition;
    try {
        tf.transformPoint("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPosition);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Failed to transform absolute goal to base footprint");
        as.setAborted();
        return;
    }
    
    // Loop and move the robot, checking the position of the goal in the base
    // frame after each move.
    ros::Rate r(10); // 10hz
    
    // How close to the goal we need to get.
    const double DISTANCE_THRESHOLD = 0.1;
    
    // Robot is at 0,0
    geometry_msgs::Point robotPosition;
    robotPosition.x = robotPosition.y = robotPosition.z = 0.0;
    
    // Loop and move the robot until the robot reaches the goal point.
    while(ros::ok() && as.isActive() && utils::pointToPointXYDistance(goalPosition.point, robotPosition) > DISTANCE_THRESHOLD){
        ROS_DEBUG("Distance to goal: %f", utils::pointToPointXYDistance(goalPosition.point, robotPosition));
        
        // Move the robot.
        btVector3 goalVector(goalPosition.point.x, goalPosition.point.y, 0);
        btScalar yaw = btAtan2(goalVector.y(), goalVector.x());
        
        // Calculate the movement.
        geometry_msgs::Twist baseCmd;

        const double MAX_V = 2.5;
        const double DEACC_DISTANCE = 0.75;
        const double MAX_REVERSE_DISTANCE = 0.5;
        
        // Robot location is at the root of frame.
        double distance = goalVector.distance(btVector3(0, 0, 0));
        if(distance > DEACC_DISTANCE){
          baseCmd.linear.x = MAX_V;
        }
        else {
          baseCmd.linear.x = distance / DEACC_DISTANCE * MAX_V;
        }

        // Determine if we should go backwards.
        // TODO: Might need to tune this further to perform the least amount of turning.
        if(goalPosition.point.x < 0){
            if(goalPosition.point.x > -MAX_REVERSE_DISTANCE){
                if(yaw > PI){
                    baseCmd.angular.z = yaw - PI;
                } else {
                    baseCmd.angular.z = yaw + PI;
                }
                baseCmd.linear.x *= -1;
                ROS_INFO("Moving in reverse at angle %f at velocity %f", baseCmd.angular.z, baseCmd.linear.x);
            }
            else {
                ROS_DEBUG("Distance too far to move in reverse");
                baseCmd.angular.z = yaw;
                baseCmd.linear.x = 0;
            }
        }
        else if(fabs(yaw) > PI / 4.0){
            ROS_DEBUG("Too far to turn and move: %f", yaw);
            baseCmd.linear.x = 0;
            baseCmd.angular.z = yaw;
        }
        else {
            baseCmd.angular.z = yaw;      
        }
        ROS_DEBUG("Moving base x: %f, y: %f, angular z: %f", baseCmd.linear.x, baseCmd.linear.y, baseCmd.angular.z);
            
        if(movePub.getNumSubscribers() > 0){
            // Visualize the movement.
            std_msgs::Header arrowHeader;
            arrowHeader.frame_id = "/base_footprint";
            arrowHeader.stamp = ros::Time::now();
            const std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
            movePub.publish(utils::createArrow(yaw, arrowHeader, GREEN));
        }
        // Publish the command to the base
        cmdVelocityPub.publish(baseCmd);

        if(nh.ok() && as.isActive()){
            r.sleep();
            // Determine the updated position of the robot.
            try {
                tf.transformPoint("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPosition);
            }
            catch(tf::TransformException& ex){
                ROS_INFO("Failed to transform absolute goal to base footprint");
                // Continue and hope that it transforms next time.
            }
        }
    }
    
    if(as.isActive()){
        stop();
        ROS_DEBUG("Move robot completed. Goal achieved.");
        as.setSucceeded();
    }
    else {
        ROS_DEBUG("Move aborted prior to completion");
    }
  }

  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::MoveRobotAction> as;
    string actionName;
    
    //! We will be listening to TF transforms
    tf::TransformListener tf;

    //! Publisher for goals
    ros::Publisher goalPub;

    //! Publisher for movement
    ros::Publisher movePub;
    
    //! Publisher for command velocities
    ros::Publisher cmdVelocityPub;
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_robot_action");
  MoveRobotAction action(ros::this_node::getName());
  ROS_INFO("Waiting for move_robot actions");
  ros::spin();
  return 0;
}
