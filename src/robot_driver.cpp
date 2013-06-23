#include <iostream>
#include <dogsim/lissajous.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

static const double PI = boost::math::constants::pi<double>();

class RobotDriver {
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmdVelocityPub_;

  //! We will be listening to TF transforms as well
  tf::TransformListener tf_;

  //! Publisher for goals
  ros::Publisher goalPub_;

  //! Publisher for movement
  ros::Publisher movePub_;

  //! Drives adjustments
  ros::Timer driverTimer;
public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh):nh_(nh){

    // Set up the publisher for the cmd_vel topic
    cmdVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);

    // Setup a visualization publisher for our goals.
    goalPub_ = nh.advertise<visualization_msgs::Marker>("path_goal", 1);
    movePub_ = nh.advertise<visualization_msgs::Marker>("planned_move", 1);

    driverTimer = nh.createTimer(ros::Duration(0.25), &RobotDriver::callback, this);
    driverTimer.start();
  }

  void callback(const ros::TimerEvent& event){
      ROS_DEBUG("Received callback @ %f", event.current_real.toSec());

      // Lookup the current position.
      bool gotTransform = false;
      const ros::Time startTime = event.current_real;
      for(unsigned int i = 0; i < 5 && !gotTransform; ++i){
        gotTransform = tf_.waitForTransform("/base_footprint", "/map", 
                                            startTime, ros::Duration(1.0));
      }
      if(!gotTransform){
        ROS_WARN("Failed to get transform. Aborting cycle");
        return;
      }
      
      // Calculate our goal.
      geometry_msgs::PointStamped goal;

      // TODO: Be smarter about making time scale based on velocity.
      gazebo::math::Vector3 gazeboGoal = lissajous::lissajous(startTime.toSec() / 50.0);
      goal.point.x = gazeboGoal.x;
      goal.point.y = gazeboGoal.y;
      goal.point.z = gazeboGoal.z;
      goal.header.stamp = startTime;
      goal.header.frame_id = "/map";
    
      // Visualize the goal.
      visualization_msgs::Marker marker;
      marker.header = goal.header;
      marker.ns = "dogsim";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = goal.point;
      marker.color.a = 1;
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.2;

      goalPub_.publish(marker);

      // Determine the angle from the robot to the target.
      geometry_msgs::PointStamped normalStamped;

      tf_.transformPoint("/base_footprint", goal, normalStamped);

      // atan gives us the yaw between the positive x axis and the point.
      double yaw = atan2(normalStamped.point.y, normalStamped.point.x);
      
      geometry_msgs::Twist baseCmd;

      bool shouldMove = true;
      const double MAX_V = 2.0;
      const double DEACC_DISTANCE = 1.0;
      const double DISTANCE_THRESH = 0.01;

      // TODO: This should use real distance, not x which won't account
      //       for y distance.
      // if(abs(normalStamped.vector.x) > DEACC_DISTANCE){
      //   baseCmd.linear.x = MAX_V;
      //}
      // Don't attempt to close on the target
      //else if(abs(normalStamped.vector.x) < DISTANCE_THRESH){
      //  shouldMove = false;
      //}
      //else {
      //  baseCmd.linear.x = abs(normalStamped.vector.x) / DEACC_DISTANCE * MAX_V;
      //}
      baseCmd.linear.x = MAX_V;
      baseCmd.linear.y = 0;
      baseCmd.angular.z = yaw;
      
      if(shouldMove){
        // Publish a visualization arrow.
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "base_footprint";
        arrow.header.stamp = startTime;
        arrow.ns = "dogsim";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose.position.x = arrow.pose.position.y = arrow.pose.position.z = 0;
        arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
        arrow.scale.x = arrow.scale.y = arrow.scale.z = 1.0;
        arrow.color.a = 1;
        arrow.color.r = 1;
        arrow.color.g = 1;
        arrow.color.b = 0;

        // TODO: Show velocity as arrow length
        movePub_.publish(arrow);
        cmdVelocityPub_.publish(baseCmd);
      }
      else {
        ROS_DEBUG("Skipping movement. Below distance tolerance");
      } 
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  ROS_INFO("Spinning the event loop");
  ros::spin();
  ROS_INFO("Exiting");
}
