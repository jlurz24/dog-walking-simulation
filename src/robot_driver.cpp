#include <ros/ros.h>
#include <dogsim/utils.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>

#include <message_filters/subscriber.h>
#include <dogsim/AdjustDogPositionAction.h>
#include <dogsim/AdjustBasePositionAction.h>
#include <dogsim/MoveRobotAction.h>
#include <dogsim/MoveDogAwayAction.h>
#include <actionlib/client/simple_action_client.h>

namespace {
  using namespace std;
  typedef actionlib::SimpleActionClient<dogsim::AdjustDogPositionAction> AdjustDogClient;
  typedef actionlib::SimpleActionClient<dogsim::AdjustBasePositionAction> AdjustBaseClient;
  typedef actionlib::SimpleActionClient<dogsim::MoveRobotAction> MoveRobotClient;
  typedef actionlib::SimpleActionClient<dogsim::MoveDogAwayAction> MoveDogAwayClient;
class RobotDriver {
private:
  //! Amount of time before starting walk
  static const double DELAY_TIME = 2.5;
  
  //! Radius of the robot to edge of the square base.
  static const double BASE_RADIUS = 0.668 / 2.0;

  //! Amount of time in the "future" to operate
  static const double FUTURE_DELTA_T = 0.5;
  
  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! Private nh
  ros::NodeHandle pnh_;

  //! Publisher for command velocities
  ros::Publisher cmdVelocityPub_;

  //! Publisher for goals
  ros::Publisher goalPub_;
    
  //! Publisher for future dog position
  ros::Publisher futureDogPosPub_;

  //! Timer that controls movement of the robot (in solo mode only)
  ros::Timer driverTimer_;
  
  //! Timer that display the goal
  ros::Timer displayTimer_;
  
  //! One shot timer that performs delayed start
  ros::Timer initTimer_;
  
  //! Dog position subscriber
  auto_ptr<message_filters::Subscriber<dogsim::DogPosition> > dogPositionSub_;
  
  //! Client for the arm to attempt to position the dog
  AdjustDogClient adjustDogClient;
  
  //! Client to adjust the position of the robot.
  AdjustBaseClient adjustBaseClient;
  
  //! Client for the movement of the robot base. Used for solo mode only
  MoveRobotClient moveRobotClient;
  
  //! Client for moving the dog out of the way
  MoveDogAwayClient moveDogAwayClient;
  
  //! Cached service client.
  ros::ServiceClient getPathClient;
  
  //! Length of the leash
  double leashLength;
  
  //! Whether the robot is operating on its own
  bool soloMode_;
  
public:
  //! ROS node initialization
  RobotDriver(): pnh_("~"), adjustDogClient("adjust_dog_position_action", true),
                            adjustBaseClient("adjust_base_position_action", true),
                            moveRobotClient("move_robot_action", true),
                            moveDogAwayClient("move_dog_away_action", true){      
    ROS_INFO("Initializing the robot driver @ %f", ros::Time::now().toSec());
    
    nh_.param("leash_length", leashLength, 2.0);
    
    dogPositionSub_.reset(new message_filters::Subscriber<dogsim::DogPosition> (nh_, "/dog_position", 1));

    // Set up the publisher
    cmdVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
    goalPub_ = nh_.advertise<visualization_msgs::Marker>("robot_driver/walk_goal_viz", 1);
    futureDogPosPub_ = nh_.advertise<visualization_msgs::Marker>("robot_driver/future_dog_position_viz", 1);
    
    ros::service::waitForService("/dogsim/get_path");
    ros::service::waitForService("/dogsim/start");
    getPathClient = nh_.serviceClient<dogsim::GetPath>("/dogsim/get_path", true /* persist */);
    dogPositionSub_->registerCallback(boost::bind(&RobotDriver::dogPositionCallback, this, _1));
    
    moveRobotClient.waitForServer();
    
    // Only use the steering callback when in solo mode. Otherwise we'll move based on the required positions to
    // move the arm.
    pnh_.param<bool>("solo_mode", soloMode_, false);
    if(soloMode_){
        ROS_INFO("Running solo mode");
    }
    else {
      ROS_INFO("Running regular mode");
      adjustDogClient.waitForServer();
      adjustBaseClient.waitForServer();
      moveDogAwayClient.waitForServer();
    }

    initTimer_ = nh_.createTimer(ros::Duration(DELAY_TIME), &RobotDriver::init, this, true /* One shot */);
    ROS_INFO("Robot driver initialization complete @ %f", ros::Time::now().toSec());
  }

  void init(const ros::TimerEvent& event){
    ROS_INFO("Entering delayed init");
    startPath(event.current_real);
    displayTimer_ = nh_.createTimer(ros::Duration(0.2), &RobotDriver::displayCallback, this);
    
    if(soloMode_){
        driverTimer_ = nh_.createTimer(ros::Duration(1.0), &RobotDriver::steeringCallback, this);
    }
    ROS_INFO("Delayed init complete");
  }
  
    void startPath(const ros::Time& currentTime){
        dogsim::StartPath startPath;
        startPath.request.time = currentTime.toSec();
        ros::ServiceClient startPathClient = nh_.serviceClient<dogsim::StartPath>("/dogsim/start", false);
        startPathClient.call(startPath);
    }
    
  void dogPositionCallback(const dogsim::DogPositionConstPtr& dogPosition){
    ROS_DEBUG("Received a dog position callback @ %f", ros::Time::now().toSec());
    
    bool ended = false;
    bool started = false;
    const geometry_msgs::PointStamped goalCurrent = getDogPosition(ros::Time(ros::Time::now().toSec()), started, ended);
    if(!started || ended){
        // TODO: Could subscribe and unsubscribe later.
        return;
    }

    const geometry_msgs::PointStamped futureGoal = getDogPosition(ros::Time(ros::Time::now().toSec() + FUTURE_DELTA_T), started, ended);

    // Assumes message is in robot frame
    assert(dogPosition->pose.header.frame_id == "/base_footprint");
    
    const double FRONT_AVOIDANCE_THRESHOLD = 0.50;
    
    // Calculate the future position.
    geometry_msgs::PoseStamped expectedDogPosition;
    expectedDogPosition.header = dogPosition->header;
    expectedDogPosition.pose.position.x = dogPosition->pose.pose.position.x + dogPosition->twist.twist.linear.x * FUTURE_DELTA_T;
    expectedDogPosition.pose.position.y = dogPosition->pose.pose.position.y + dogPosition->twist.twist.linear.y * FUTURE_DELTA_T;
    expectedDogPosition.pose.position.z = dogPosition->pose.pose.position.z + dogPosition->twist.twist.linear.z * FUTURE_DELTA_T;
    
    // Visualize the future position
    if(futureDogPosPub_.getNumSubscribers() > 0){
        std_msgs::ColorRGBA PURPLE = utils::createColor(0.5, 0, 0.5);
        futureDogPosPub_.publish(utils::createMarker(expectedDogPosition.pose.position, expectedDogPosition.header, PURPLE, false));
    }
      
    // Determine if our base movement should be to avoid the dog. First priority.
    // TODO: Move this to separate action so it can more intelligently avoid the dog. Potentially.
    if(dogPosition->pose.pose.position.x < FRONT_AVOIDANCE_THRESHOLD && dogPosition->pose.pose.position.x >= BASE_RADIUS && abs(dogPosition->pose.pose.position.y) < BASE_RADIUS){
      ROS_INFO("Attempting to avoid dog @ %f %f with FAT %f and BR = %f", dogPosition->pose.pose.position.x, dogPosition->pose.pose.position.y, FRONT_AVOIDANCE_THRESHOLD, BASE_RADIUS);
      
      // Stop current movement.
      adjustDogClient.cancelGoal();
      adjustBaseClient.cancelGoal();
      
      dogsim::MoveDogAwayGoal goal;
      moveDogAwayClient.sendGoal(goal);
      return;
    }
    
    // Only adjust dog position if the last adjustment finished
    if(adjustDogClient.getState() != actionlib::SimpleClientGoalState::ACTIVE){
        dogsim::AdjustDogPositionGoal adjustGoal;
        adjustGoal.dogPose = dogPosition->pose;
        adjustGoal.goalPosition = goalCurrent;
        adjustGoal.futureDogPose = expectedDogPosition;
        adjustGoal.futureGoalPosition = futureGoal;
        adjustDogClient.sendGoal(adjustGoal);
    }
    if(adjustBaseClient.getState() != actionlib::SimpleClientGoalState::ACTIVE){
        dogsim::AdjustBasePositionGoal adjustGoal;
        adjustGoal.dogPose = dogPosition->pose;
        adjustGoal.goalPosition = goalCurrent;
        adjustGoal.futureDogPose = expectedDogPosition;
        adjustGoal.futureGoalPosition = futureGoal;
        adjustBaseClient.sendGoal(adjustGoal);
    } 
    ROS_DEBUG("Completed dog position callback");
  }

  geometry_msgs::PointStamped getDogPosition(const ros::Time& time, bool& started, bool& ended){
      // Determine the goal.
      dogsim::GetPath getPath;
      getPath.request.time = time.toSec();
      getPathClient.call(getPath);
      started = getPath.response.started;
      ended = getPath.response.ended;
      return getPath.response.point;
  }

  void displayCallback(const ros::TimerEvent& event){
      bool started;
      bool ended;
      const geometry_msgs::PointStamped goal = getDogPosition(event.current_real, started, ended);
      assert(started);
      
      if(ended){
          ROS_INFO("Walk is ended");
          driverTimer_.stop();
          return;
      }

      // Visualize the goal.
      if(goalPub_.getNumSubscribers() > 0){
          ROS_DEBUG("Publishing the goal position");
          std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
          goalPub_.publish(utils::createMarker(goal.point, goal.header, RED, true));
      }
  }
  
  void steeringCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received callback @ %f : %f", event.current_real.toSec(), event.current_expected.toSec());


      bool ended = false;
      bool started = false;
      const geometry_msgs::PointStamped goal = getDogPosition(event.current_real, started, ended);

      if(ended){
          ROS_INFO("Walk ended");
          if(moveRobotClient.getState() == actionlib::SimpleClientGoalState::ACTIVE){
              moveRobotClient.cancelGoal();
          }
          driverTimer_.stop();
          return;
      }
      
      dogsim::MoveRobotGoal moveRobotGoal;
      moveRobotGoal.position = goal;
      
      // This will automatically cancel the last goal.
      moveRobotClient.sendGoal(moveRobotGoal);
      ROS_DEBUG("Completed robot driver callback");
    }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_driver");

  RobotDriver driver;
  ros::spin();
  ROS_INFO("Exiting robot driver");
}
