#include <ros/ros.h>
#include <dogsim/utils.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>

#include <message_filters/subscriber.h>
#include <dogsim/AdjustDogPositionAction.h>
#include <dogsim/MoveArmToBasePositionAction.h>
#include <dogsim/MoveRobotAction.h>
#include <dogsim/MoveDogAwayAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/btVector3.h>

namespace {
  using namespace std;
  using namespace dogsim;
  
  typedef actionlib::SimpleActionClient<AdjustDogPositionAction> AdjustDogClient;
  typedef actionlib::SimpleActionClient<MoveRobotAction> MoveRobotClient;
  typedef actionlib::SimpleActionClient<MoveDogAwayAction> MoveDogAwayClient;
  typedef actionlib::SimpleActionClient<MoveArmToBasePositionAction> MoveArmToBasePositionClient;
class RobotDriver {
private:
  //! Amount of time before starting walk
  static const double DELAY_TIME = 2.5;
  
  //! Radius of the robot to edge of the square base.
  static const double BASE_RADIUS = 0.668 / 2.0;
  
  //! Space to keep in front of the robot in meters
  static const double FRONT_AVOIDANCE_THRESHOLD = 0.50;
  
  //! Slope delta for calculating robot path.
  static const double SLOPE_DELTA = 0.01;
  
  // Calculated as the negative of /base_footprint to /r_wrist_roll_link in x axis
  static const double TRAILING_DISTANCE = 0.585;
  
  //! Shift distance from base to desired arm position
  //! Calculated as the negative of /base_footprint to /r_wrist_roll_link in x axis
  //! TODO: This may need further calibration
  static const double SHIFT_DISTANCE = 0.6;
  
  //! Amount of time in the future to operate
  double knownPositionWindow;
  
  //! Node handle
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
  auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub_;
  
  //! Client for the arm to attempt to position the dog
  AdjustDogClient adjustDogClient;
  
  //! Client for the movement of the robot base.
  MoveRobotClient moveRobotClient;
  
  //! Client for moving the dog out of the way
  MoveDogAwayClient moveDogAwayClient;
  
  //! Client to adjust the arm of the robot to the start position
  MoveArmToBasePositionClient moveArmToBasePositionClient;
  
  //! Cached service client.
  ros::ServiceClient getPathClient;
  
  //! Length of the leash
  double leashLength;
  
  //! Whether the robot is operating on its own
  bool soloMode;
  
  //! Transform listener
  tf::TransformListener tf;
  
public:
  //! ROS node initialization
  RobotDriver(): pnh_("~"), adjustDogClient("adjust_dog_position_action", true),
                            moveRobotClient("move_robot_action", true),
                            moveDogAwayClient("move_dog_away_action", true),
                            moveArmToBasePositionClient("move_arm_to_base_position_action", true){      
    ROS_INFO("Initializing the robot driver @ %f", ros::Time::now().toSec());
    
    nh_.param("leash_length", leashLength, 2.0);
    nh_.param("dog_position_detector/known_position_window", knownPositionWindow, 0.0);
    
    dogPositionSub_.reset(new message_filters::Subscriber<DogPosition> (nh_, "/dog_position", 1));

    // Set up the publisher
    cmdVelocityPub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
    goalPub_ = nh_.advertise<visualization_msgs::Marker>("robot_driver/walk_goal_viz", 1);
    futureDogPosPub_ = nh_.advertise<visualization_msgs::Marker>("robot_driver/future_dog_position_viz", 1);
    
    ros::service::waitForService("/dogsim/get_path");
    ros::service::waitForService("/dogsim/start");
    getPathClient = nh_.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);
    dogPositionSub_->registerCallback(boost::bind(&RobotDriver::dogPositionCallback, this, _1));
    
    moveRobotClient.waitForServer();
    ROS_INFO("Waiting for move arm to base position client");
    moveArmToBasePositionClient.waitForServer();
    ROS_INFO("Completed waiting");
    
    // Only use the steering callback when in solo mode. Otherwise we'll move based on the required positions to
    // move the arm.
    pnh_.param<bool>("solo_mode", soloMode, false);
    if(soloMode){
        ROS_INFO("Running solo mode");
    }
    else {
      ROS_INFO("Running regular mode");
      adjustDogClient.waitForServer();
      moveDogAwayClient.waitForServer();
    }

    initTimer_ = nh_.createTimer(ros::Duration(DELAY_TIME), &RobotDriver::init, this, true /* One shot */);
    ROS_INFO("Robot driver initialization complete @ %f", ros::Time::now().toSec());
  }

  void init(const ros::TimerEvent& event){
    ROS_INFO("Entering delayed init");
    
    MoveArmToBasePositionGoal moveArmToBasePositionGoal;
    utils::sendGoal(&moveArmToBasePositionClient, moveArmToBasePositionGoal, nh_, 10.0);
    
    startPath(ros::Time::now());
    displayTimer_ = nh_.createTimer(ros::Duration(0.2), &RobotDriver::displayCallback, this);
    driverTimer_ = nh_.createTimer(ros::Duration(0.5), &RobotDriver::steeringCallback, this);
    ROS_INFO("Delayed init complete");
  }
  
    void startPath(const ros::Time& currentTime){
        StartPath startPath;
        startPath.request.time = currentTime.toSec();
        ros::ServiceClient startPathClient = nh_.serviceClient<StartPath>("/dogsim/start", false);
        startPathClient.call(startPath);
    }
    
  void dogPositionCallback(const DogPositionConstPtr& dogPosition){
    ROS_DEBUG("Received a dog position callback @ %f", ros::Time::now().toSec());
    
    bool ended = false;
    bool started = false;
    const geometry_msgs::PointStamped goalCurrent = getDogGoalPosition(ros::Time(ros::Time::now().toSec()), started, ended);
    if(!started || ended){
        // TODO: Could subscribe and unsubscribe later.
        return;
    }

    const double FUTURE_DELTA_T = 2.0;
    const geometry_msgs::PointStamped futureGoal = getDogGoalPosition(ros::Time(ros::Time::now().toSec() + FUTURE_DELTA_T), started, ended);

    geometry_msgs::PoseStamped expectedDogPose;
    if(dogPosition->futurePoseKnown){
        expectedDogPose = dogPosition->futurePose;
    }
    else {
        // Calculate the future position.
        expectedDogPose.header = dogPosition->header;
        expectedDogPose.pose.position.x = dogPosition->pose.pose.position.x + dogPosition->twist.twist.linear.x * FUTURE_DELTA_T;
        expectedDogPose.pose.position.y = dogPosition->pose.pose.position.y + dogPosition->twist.twist.linear.y * FUTURE_DELTA_T;
        expectedDogPose.pose.position.z = dogPosition->pose.pose.position.z + dogPosition->twist.twist.linear.z * FUTURE_DELTA_T;
    }
    
    // Visualize the future position
    if(futureDogPosPub_.getNumSubscribers() > 0){
        std_msgs::ColorRGBA PURPLE = utils::createColor(0.5, 0, 0.5);
        futureDogPosPub_.publish(utils::createMarker(expectedDogPose.pose.position, expectedDogPose.header, PURPLE, false));
    }
    
    // Convert the positions to the robot frame.
    geometry_msgs::PoseStamped dogPoseInBaseFrame;
    tf.transformPose("/base_footprint", ros::Time(0), dogPosition->pose, dogPosition->pose.header.frame_id, dogPoseInBaseFrame);
    
    geometry_msgs::PoseStamped expectedDogPoseInBaseFrame;
    tf.transformPose("/base_footprint", ros::Time(0), expectedDogPose, expectedDogPose.header.frame_id, expectedDogPoseInBaseFrame);
        
    // Determine if our base movement should be to avoid the dog. First priority.
    // TODO: Move this to separate action so it can more intelligently avoid the dog. Potentially.
    // TODO: This makes more sense to me as a separate node that listens for this event and then sends 
    //       an avoiding dog event.
    if(dogPoseInBaseFrame.pose.position.x < FRONT_AVOIDANCE_THRESHOLD && dogPoseInBaseFrame.pose.position.x >= BASE_RADIUS && abs(dogPoseInBaseFrame.pose.position.y) < BASE_RADIUS){
      ROS_INFO("Attempting to avoid dog @ %f %f with FAT %f and BR = %f", dogPoseInBaseFrame.pose.position.x, dogPoseInBaseFrame.pose.position.y, FRONT_AVOIDANCE_THRESHOLD, BASE_RADIUS);
      
      // Stop current movement.
      adjustDogClient.cancelGoal();
      moveRobotClient.cancelGoal();
      
      MoveDogAwayGoal goal;
      moveDogAwayClient.sendGoal(goal);
      return;
    }
    
    // Only adjust dog position if the last adjustment finished
    if(adjustDogClient.getState() != actionlib::SimpleClientGoalState::ACTIVE){
        AdjustDogPositionGoal adjustGoal;
        adjustGoal.dogPose = dogPoseInBaseFrame;
        adjustGoal.goalPosition = goalCurrent;
        adjustGoal.futureDogPose = expectedDogPoseInBaseFrame;
        adjustGoal.futureGoalPosition = futureGoal;
        adjustDogClient.sendGoal(adjustGoal);
    }
    ROS_DEBUG("Completed dog position callback");
  }

  geometry_msgs::PointStamped getDogGoalPosition(const ros::Time& time, bool& started, bool& ended){
      // Determine the goal.
      GetPath getPath;
      getPath.request.time = time.toSec();
      getPathClient.call(getPath);
      started = getPath.response.started;
      ended = getPath.response.ended;
      return getPath.response.point;
  }

  void displayCallback(const ros::TimerEvent& event){
      if(goalPub_.getNumSubscribers() > 0){
        bool started;
        bool ended;
        const geometry_msgs::PointStamped goal = getDogGoalPosition(event.current_real, started, ended);
        assert(started);
      
        if(ended){
            ROS_INFO("Walk is ended");
            displayTimer_.stop();
            return;
        }

        // Visualize the goal.
        ROS_DEBUG("Publishing the goal position");
        std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
        goalPub_.publish(utils::createMarker(goal.point, goal.header, RED, true));
      }
  }
  
  void steeringCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received callback @ %f : %f", event.current_real.toSec(), event.current_expected.toSec());

      bool ended = false;
      bool started = false;
      const geometry_msgs::PointStamped dogGoal = getDogGoalPosition(event.current_real, started, ended);

      if(ended){
          ROS_INFO("Walk ended");
          if(moveRobotClient.getState() == actionlib::SimpleClientGoalState::ACTIVE){
              moveRobotClient.cancelGoal();
          }
          driverTimer_.stop();
          return;
      }
      
      const geometry_msgs::PointStamped goal2 = getDogGoalPosition(ros::Time(event.current_real.toSec() + SLOPE_DELTA), started, ended);
      
      // Calculate the vector of the tangent line.
      btVector3 tangent = btVector3(goal2.point.x, goal2.point.y, 0) - btVector3(dogGoal.point.x, dogGoal.point.y, 0);
      tangent.normalize();
 
      // Now select a point on the vector but slightly behind.
      btVector3 backGoal = btVector3(dogGoal.point.x, dogGoal.point.y, 0) - tangent * btScalar(TRAILING_DISTANCE);
   
      // Rotate the vector to perpendicular
      btVector3 perp = backGoal.normalized().rotate(btVector3(0, 0, 1), btScalar(boost::math::constants::pi<double>() / 2.0));

      // Select a point on the perpindicular line.
      btVector3 finalGoal = backGoal + perp * btScalar(SHIFT_DISTANCE);
  
      geometry_msgs::PointStamped robotGoal;
      robotGoal.header = dogGoal.header;
      robotGoal.point.x = finalGoal.x();
      robotGoal.point.y = finalGoal.y();
      robotGoal.point.z = finalGoal.z();

      MoveRobotGoal moveRobotGoal;
      moveRobotGoal.position = robotGoal;
      
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
