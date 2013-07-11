#include <iostream>
#include <dogsim/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/GetModelState.h>

namespace {
  using namespace std;
class RobotDriver {
private:
  //! Length of the leash. Keep this in-sync with the leash controller.
  static const double LEASH_LENGTH = 3.0;

  //! Amount of time it takes to perform a full lissajous cycle.
  static const double LISSAJOUS_FULL_CYCLE_T = 4.45;

  //! Amount of time before starting walk
  static const double DELAY_TIME = 1.0;

  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmdVelocityPub_;

  //! We will be listening to TF transforms as well
  tf::TransformListener tf_;

  //! Publisher for goals
  ros::Publisher goalPub_;

  //! Publisher for the dog position.
  ros::Publisher dogPub_;

  //! Publisher for movement
  ros::Publisher movePub_;

  //! Publisher for the trailing point
  ros::Publisher trailingPointPub_;

  //! Drives adjustments
  ros::Timer driverTimer;

  //! Time the driver was initialized
  ros::Time initTime;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh):nh_(nh){

    // Set up the publisher for the cmd_vel topic
    cmdVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);

    // Setup a visualization publisher for our goals.
    goalPub_ = nh_.advertise<visualization_msgs::Marker>("path_goal", 1);
    movePub_ = nh_.advertise<visualization_msgs::Marker>("planned_move", 1);
    dogPub_ = nh_.advertise<visualization_msgs::Marker>("dog_position", 1);
    trailingPointPub_ = nh_.advertise<visualization_msgs::Marker>("trailing_point", 1);

    driverTimer = nh_.createTimer(ros::Duration(0.1), &RobotDriver::callback, this);
    // Wait for the service that will provide us simulated object locations.
    ros::service::waitForService("/gazebo/get_model_state");

    initTime = ros::Time::now();
    driverTimer.start();
  }

  static visualization_msgs::Marker createMarker(const geometry_msgs::Point& position, const std_msgs::Header& header, std_msgs::ColorRGBA& color){

      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "dogsim";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = position;
      marker.color = color;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
      return marker;
  }

  static std_msgs::ColorRGBA createColor(float r, float g, float b){
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    return color;
  }

  static visualization_msgs::Marker createArrow(const double yaw, const std_msgs::Header& header, const std_msgs::ColorRGBA& color){
     // Publish a visualization arrow.
     visualization_msgs::Marker arrow;
     arrow.header = header;
     arrow.ns = "dogsim";
     arrow.id = 0;
     arrow.type = visualization_msgs::Marker::ARROW;
     arrow.action = visualization_msgs::Marker::ADD;
     arrow.pose.position.x = arrow.pose.position.y = arrow.pose.position.z = 0;
     arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
     arrow.scale.x = arrow.scale.y = arrow.scale.z = 1.0;
     arrow.color = color;
     return arrow;
 }

  void callback(const ros::TimerEvent& event){
      ROS_DEBUG("Received callback @ %f", event.current_real.toSec());

      if(event.current_real.toSec() - initTime.toSec() < DELAY_TIME){
        ROS_DEBUG("Start time not reached");
        return;
      }

      // Lookup the current position.
      bool gotTransform = false;
      for(unsigned int i = 0; i < 2 && !gotTransform; ++i){
        gotTransform = tf_.waitForTransform("/base_footprint", "/map", 
                                            event.current_real, ros::Duration(1.0));
      }
      if(!gotTransform){
        ROS_WARN("Failed to get transform. Aborting cycle");
        return;
      }

      // Set the start time of all goals.
      bool isStarted;
      nh_.param<bool>("path/started", isStarted, false);
      double startTime = event.current_real.toSec();
      if(!isStarted){
        ROS_INFO("Setting start time to %f", event.current_real.toSec());
        nh_.setParam("path/start_time", event.current_real.toSec());
        nh_.setParam("path/started", true);
      }
      else {
        nh_.getParam("path/start_time", startTime);
      }
      
      bool isEnded;
      nh_.param<bool>("path/ended", isEnded, false);
      if(isEnded){
        return;
      }
      if((event.current_real.toSec() - startTime)/ utils::TIMESCALE_FACTOR > LISSAJOUS_FULL_CYCLE_T){
        ROS_INFO("Setting end flag");
        nh_.setParam("path/ended", true);
        return;
      }
      // Lookup the current position of the dog.
      ros::ServiceClient modelStateServ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gazebo_msgs::GetModelState modelState;
      modelState.request.model_name = "dog";
      modelStateServ.call(modelState);
      geometry_msgs::PoseStamped dogPose;
      dogPose.header.stamp = event.current_real;
      dogPose.header.frame_id = "/map";
      dogPose.pose = modelState.response.pose;
    
      // Visualize the dog.
      std_msgs::ColorRGBA BLUE = createColor(0, 0, 1);
      dogPub_.publish(createMarker(dogPose.pose.position, dogPose.header, BLUE));

      // Determine the goal.
      geometry_msgs::PointStamped goal;

      // TODO: Be smarter about making time scale based on velocity.
      gazebo::math::Vector3 gazeboGoal = utils::lissajous((event.current_real.toSec() - startTime) / utils::TIMESCALE_FACTOR);

      goal.point.x = gazeboGoal.x;
      goal.point.y = gazeboGoal.y;
      goal.point.z = gazeboGoal.z;
      goal.header.stamp = event.current_real;
      goal.header.frame_id = "/map";
      
      // Visualize the goal.
      std_msgs::ColorRGBA RED = createColor(1, 0, 0);
      goalPub_.publish(createMarker(goal.point, goal.header, RED));

      // Determine the angle from the robot to the target.
      geometry_msgs::PointStamped normalStamped;
      tf_.transformPoint("/base_footprint", goal, normalStamped);

      // Determine the relative dog position
      geometry_msgs::PoseStamped dogInBaseFrame;
      tf_.transformPose("/base_footprint", dogPose, dogInBaseFrame);

      // Now calculate a point that is the leash/2 length distance behind the dog
      // on the vector between the robot and the dog. 
      // This is the desired position of the robot.
      btVector3 goalVector(normalStamped.point.x, normalStamped.point.y, 0);
      goalVector -= btScalar(LEASH_LENGTH / 2) * goalVector.normalized();

      // Publish the trailing point.
      std_msgs::ColorRGBA PURPLE = createColor(0.5, 0.0, 0.5);
      geometry_msgs::PointStamped trailingPoint;
      trailingPoint.header.frame_id = "/base_footprint";
      trailingPoint.header.stamp = event.current_real;
      trailingPoint.point.x = goalVector.x();
      trailingPoint.point.y = goalVector.y();
      trailingPoint.point.z = 0;
      trailingPointPub_.publish(createMarker(trailingPoint.point, trailingPoint.header, PURPLE));

      // atan gives us the yaw between the positive x axis and the point.
      btScalar yaw = btAtan2(goalVector.y(), goalVector.x());
      
      // Calculate the movement.
      geometry_msgs::Twist baseCmd;

      bool shouldMove = true;
      const double MAX_V = 3;
      const double AVOIDANCE_V = 5;
      const double AVOIDANCE_THRESHOLD = 1.0;
      const double DEACC_DISTANCE = 0.5;
      const double MIN_V = 0.6;

      // Robot location is at the root of frame.
      double distance = goalVector.distance(btVector3(0, 0, 0));
      if(distance > DEACC_DISTANCE){
        baseCmd.linear.x = MAX_V;
      }
      else {
        baseCmd.linear.x = distance / DEACC_DISTANCE * MAX_V;
      }
      
      // Check if the velocity would be below our minimum. This prevents very
      // small movements that might cause us to switch directions.
      if(baseCmd.linear.x < MIN_V){
        shouldMove = false;
      }
      
      // Check if we are likely to collide with the dog and go around it.
      if(abs(dogInBaseFrame.pose.position.y) < AVOIDANCE_THRESHOLD && (dogInBaseFrame.pose.position.x > 0 && dogInBaseFrame.pose.position.x < AVOIDANCE_THRESHOLD)){
        ROS_INFO("Attempting to avoid dog");
        // Move in the opposite direction of the position of the dog.
        baseCmd.linear.y = -1.0 * copysign(AVOIDANCE_V, dogInBaseFrame.pose.position.y);
      }
      // We will naturally correct back to the goal.
      
      baseCmd.angular.z = yaw;
      
      if(shouldMove){
        ROS_INFO("Moving base x: %f, y: %f, z: %f", baseCmd.linear.x, baseCmd.linear.y, baseCmd.angular.z);
        // Visualize the movement.
        std_msgs::Header arrowHeader;
        arrowHeader.frame_id = "base_footprint";
        arrowHeader.stamp = event.current_real;
        const std_msgs::ColorRGBA GREEN = createColor(0, 1, 0);
        movePub_.publish(createArrow(yaw, arrowHeader, GREEN));
        
        // Publish the command to the base
        cmdVelocityPub_.publish(baseCmd);
      }
      else {
        ROS_INFO("Skipping movement. Below distance tolerance");
      } 
    }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  ROS_INFO("Spinning the event loop");
  ros::spin();
  ROS_INFO("Exiting");
}
