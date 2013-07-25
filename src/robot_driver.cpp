#include <ros/ros.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetPath.h>
#include <message_filters/subscriber.h>

namespace {
  using namespace std;
class RobotDriver {
private:
  //! Length of the leash. Keep this in-sync with the leash controller.
  static const double LEASH_LENGTH = 2.0;

  //! Amount of time before starting walk
  static const double DELAY_TIME = 1.0;

  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! Private nh
  ros::NodeHandle pnh_;

  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmdVelocityPub_;

  //! We will be listening to TF transforms
  tf::TransformListener tf_;

  //! Publisher for goals
  ros::Publisher goalPub_;

  //! Publisher for movement
  ros::Publisher movePub_;

  //! Publisher for the trailing point
  ros::Publisher trailingPointPub_;

  //! Timer for moving the robot
  ros::Timer driverTimer_;

  //! Time the driver was initialized
  ros::Time initTime;

  //! Dog position subscriber
  auto_ptr<message_filters::Subscriber<dogsim::DogPosition> > dogPositionSub_;

public:
  //! ROS node initialization
  RobotDriver(): pnh_("~"){
    ROS_INFO("Initializing the robot driver");
    dogPositionSub_.reset(new message_filters::Subscriber<dogsim::DogPosition> (nh_, "/dog_position", 1));

    // Set up the publisher for the cmd_vel topic
    cmdVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);

    // Setup a visualization publisher for our goals.
    goalPub_ = nh_.advertise<visualization_msgs::Marker>("path_goal", 1);
    movePub_ = nh_.advertise<visualization_msgs::Marker>("planned_move", 1);
    trailingPointPub_ = nh_.advertise<visualization_msgs::Marker>("trailing_point", 1);
    
    ros::service::waitForService("/dogsim/get_path");

    initTime = ros::Time::now();
    dogPositionSub_->registerCallback(boost::bind(&RobotDriver::dogPositionCallback, this, _1));
    driverTimer_ = nh_.createTimer(ros::Duration(0.1), &RobotDriver::steeringCallback, this);
    driverTimer_.start();
    ROS_INFO("Robot driver initialization complete");
  }

  void dogPositionCallback(const dogsim::DogPositionConstPtr& dogPosition){
    ROS_INFO("Received a dog position callback");
    // Assumes messages is in robot frame
    // Check if we are likely to collide with the dog and go around it.
    const double AVOIDANCE_V = 5;
    const double AVOIDANCE_THRESHOLD = 1.0;

    if(abs(dogPosition->pose.pose.position.y) < AVOIDANCE_THRESHOLD && (dogPosition->pose.pose.position.x > 0 && dogPosition->pose.pose.position.x < AVOIDANCE_THRESHOLD)){
      ROS_INFO("Attempting to avoid dog");
      geometry_msgs::Twist baseCmd;
      // Move in the opposite direction of the position of the dog.
      baseCmd.linear.y = -1.0 * copysign(AVOIDANCE_V, dogPosition->pose.pose.position.y);
      
      // Publish the command to the base
    }
    // We will naturally correct back to the goal.
  }

  void steeringCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received callback @ %f : %f", event.current_real.toSec(), event.current_expected.toSec());
      ros::WallTime realStartTime = ros::WallTime::now();
      
      ROS_INFO("Started robot driver callback @ %f", realStartTime.toSec());

      if(event.current_real.toSec() - initTime.toSec() < DELAY_TIME){
        ROS_DEBUG("Start time not reached");
        return;
      }

      // Determine the goal.
      ros::ServiceClient getPathClient = nh_.serviceClient<dogsim::GetPath>("/dogsim/get_path");
      dogsim::GetPath getPath;
      getPath.request.time = event.current_real.toSec();
      getPath.request.start = true;
      getPathClient.call(getPath);
      if(getPath.response.ended){
        ROS_INFO("Walk ended");
        return;
      } 
      geometry_msgs::PointStamped goal = getPath.response.point;
      ROS_DEBUG("Calculated goal @ %f", ros::WallTime::now().toSec());
      
      // Visualize the goal.
      std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
      goalPub_.publish(utils::createMarker(goal.point, goal.header, RED, true));
      ROS_DEBUG("Published visualization goal @ %f", ros::WallTime::now().toSec());

      // Determine the angle from the robot to the target.
      geometry_msgs::PointStamped normalStamped;
      try {
        tf_.transformPoint("/base_footprint", ros::Time(0), goal, goal.header.frame_id, normalStamped);
      }
      catch(tf::TransformException& ex){
        ROS_INFO("Failed to transform point to base footprint");
        return;
      }
      ROS_DEBUG("Finished transforming the goal position @ %f", ros::WallTime::now().toSec());

      // Now calculate a point that is the leash/2 length distance behind the dog
      // on the vector between the robot and the dog. 
      // This is the desired position of the robot.
      btVector3 goalVector(normalStamped.point.x, normalStamped.point.y, 0);
      goalVector -= btScalar(1.5) * goalVector.normalized();
      ROS_DEBUG("Finished calculating the goal point @ %f", ros::WallTime::now().toSec());

      // Publish the trailing point.
      std_msgs::ColorRGBA PURPLE = utils::createColor(0.5, 0.0, 0.5);
      geometry_msgs::PointStamped trailingPoint;
      trailingPoint.header.frame_id = "/base_footprint";       
      trailingPoint.header.stamp = event.current_real;
      trailingPoint.point.x = goalVector.x();
      trailingPoint.point.y = goalVector.y();
      trailingPoint.point.z = 0;
      trailingPointPub_.publish(utils::createMarker(trailingPoint.point, trailingPoint.header, PURPLE, false));
      ROS_DEBUG("Finished publishing the trailing point @ %f", ros::WallTime::now().toSec());

      // atan gives us the yaw between the positive x axis and the point.
      btScalar yaw = btAtan2(goalVector.y(), goalVector.x());
      ROS_DEBUG("Calculated yaw @ %f", ros::WallTime::now().toSec());

      // Calculate the movement.
      geometry_msgs::Twist baseCmd;

      bool shouldMove = true;
      const double MAX_V = 3;
      const double DEACC_DISTANCE = 0.75;
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

      baseCmd.angular.z = yaw;      
      ROS_INFO("Calculated command @ %f", ros::WallTime::now().toSec());

      if(shouldMove){
        ROS_DEBUG("Moving base x: %f, y: %f, z: %f", baseCmd.linear.x, baseCmd.linear.y, baseCmd.angular.z);
        // Visualize the movement.
        std_msgs::Header arrowHeader;
        arrowHeader.frame_id = "base_footprint";
        arrowHeader.stamp = event.current_real;
        const std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
        movePub_.publish(utils::createArrow(yaw, arrowHeader, GREEN));
        
        // Publish the command to the base
        cmdVelocityPub_.publish(baseCmd);
      }
      else {
        ROS_DEBUG("Skipping movement. Below distance tolerance");
      }
      ROS_INFO("Completed robot driver callback @ %f with total duration %f", ros::WallTime::now().toSec(), ros::WallTime::now().toSec() - realStartTime.toSec());
    }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_driver");

  RobotDriver driver;
  ros::spin();
  ROS_INFO("Exiting robot driver");
}
