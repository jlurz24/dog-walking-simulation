#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Generated messages
#include <dogsim/MoveRobotAction.h>

namespace {
  using namespace std;

  class MoveRobotAction {
    public:
      MoveRobotAction(const string& name): as(nh, name, boost::bind(&MoveRobotAction::move, this, _1), false), actionName(name) {
        as.registerPreemptCallback(boost::bind(&MoveRobotAction::preemptCB, this));
      
        // Set up the publisher for the cmd_vel topic
        cmdVelocityPub = nh.advertise<geometry_msgs::Twist>("base_controller/command", 1);

        goalPub = nh.advertise<visualization_msgs::Marker>("move_robot_action/move_goal_viz", 1);
        movePub = nh.advertise<visualization_msgs::Marker>("move_robot_action/planned_move_viz", 1);
        as.start();
  }
  
  void preemptCB(){
    ROS_INFO("Preempting the move robot action");

    if(!as.isActive()){
      ROS_INFO("Move robot position action cancelled prior to start");
      return;
    }
    as.setPreempted();
  }

  void move(const dogsim::MoveRobotGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Move robot action cancelled prior to start");
      return;
    }
    
    // Visualize the goal.
    if(goalPub.getNumSubscribers()){
        std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
        goalPub.publish(utils::createMarker(goal->position.point, goal->position.header, RED, true));
    }
    
    // Determine the position in the map frame. This allows us to store
    // an absolute position
    geometry_msgs::PointStamped absoluteGoal;
    try {
        tf.transformPoint("/map", ros::Time(0), goal->position, goal->position.header.frame_id, absoluteGoal);
    }
    catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform goal to map frame");
        as.setAborted();
        return;
    }
    
    // Determine the updated position of the robot.
    geometry_msgs::PointStamped goalPosition;
    try {
        tf.transformPoint("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPosition);
    }
    catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform absolute goal to base footprint");
        as.setAborted();
        return;
    }
    
    // Loop and move the robot, checking the position of the goal in the base
    // frame after each move.
    ros::Rate r(10); // 25 hz
    
    // How close to the goal we need to get.
    const double DISTANCE_THRESHOLD = 0.25;
    
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

        const double MAX_V = 2.0;
        const double DEACC_DISTANCE = 1.75;

        // Robot location is at the root of frame.
        double distance = goalVector.distance(btVector3(0, 0, 0));
        if(distance > DEACC_DISTANCE){
          baseCmd.linear.x = MAX_V;
        }
        else {
          baseCmd.linear.x = distance / DEACC_DISTANCE * MAX_V;
        }

        baseCmd.angular.z = yaw;      

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
        ROS_INFO("Move robot completed. Goal achieved.");
        as.setSucceeded();
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
  ROS_INFO("Main function for move_robot_action");
  ros::init(argc, argv, "move_robot_action");
  MoveRobotAction action(ros::this_node::getName());
  ROS_INFO("Waiting for move_robot actions");
  ros::spin();
  return 0;
}
