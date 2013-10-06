#include <ros/ros.h>
#include <dogsim/utils.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/DogPosition.h>
#include <dogsim/AvoidingDog.h>
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
  typedef actionlib::SimpleActionClient<MoveArmToBasePositionAction> MoveArmToBasePositionClient;
class RobotDriver {
private:
  //! Amount of time before starting walk. This provides time for the robot to finish
  //  tucking its arms.
  static const double DELAY_TIME = 2.5;
  
  //! Slope delta for calculating robot path.
  static const double SLOPE_DELTA = 0.01;
  
  static const double TRAILING_DISTANCE = 0;
  
  //! Shift distance from base to desired arm position
  //! Calculated as the negative of /base_footprint to /r_wrist_roll_link in x axis
  static const double SHIFT_DISTANCE = 0.6;
  
  //! Interval to update the move_robot_action
  static const double MOVE_ROBOT_UPDATE_INTERVAL = 2.0;
  
  //! Node handle
  ros::NodeHandle nh;

  //! Private nh
  ros::NodeHandle pnh;
  
  //! Timer that controls movement of the robot (in solo mode only)
  ros::Timer driverTimer;
  
  //! One shot timer that performs delayed start
  ros::Timer initTimer;
  
  //! Dog position subscriber
  auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;
  
  //! Avoiding dog listener.
  auto_ptr<message_filters::Subscriber<AvoidingDog> > avoidingDogSub;
  
  //! Client for the arm to attempt to position the dog
  AdjustDogClient adjustDogClient;
  
  //! Client for the movement of the robot base.
  MoveRobotClient moveRobotClient;
  
  //! Client to adjust the arm of the robot to the start position
  MoveArmToBasePositionClient moveArmToBasePositionClient;
  
  //! Cached service client.
  ros::ServiceClient getPathClient;
  
  //! Length of the leash
  double leashLength;
  
  //! Whether the robot is operating on its own
  bool soloMode;
  
  //! Whether the robot is running in a mode where the base does not move.
  bool noSteeringMode;
  
  //! Whether are currently avoiding the dog
  bool avoidingDog;
  
public:
  //! ROS node initialization
  RobotDriver(): pnh("~"), adjustDogClient("adjust_dog_position_action", true),
                            moveRobotClient("move_robot_action", true),
                            moveArmToBasePositionClient("move_arm_to_base_position_action", true),
                            soloMode(false),
                            noSteeringMode(false),
                            avoidingDog(false){
                                
    ROS_INFO("Initializing the robot driver @ %f", ros::Time::now().toSec());
    
    nh.param("leash_length", leashLength, 2.0);
        
    ros::service::waitForService("/dogsim/get_path");
    ros::service::waitForService("/dogsim/start");
    getPathClient = nh.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);
    
    dogPositionSub.reset(new message_filters::Subscriber<DogPosition> (nh, "/dog_position_detector/dog_position", 1));
    dogPositionSub->registerCallback(boost::bind(&RobotDriver::dogPositionCallback, this, _1));
    
    avoidingDogSub.reset(new message_filters::Subscriber<AvoidingDog> (nh, "/avoid_dog/avoiding", 1));
    avoidingDogSub->registerCallback(boost::bind(&RobotDriver::avoidingDogCallback, this, _1));
    
    moveRobotClient.waitForServer();
    moveArmToBasePositionClient.waitForServer();
    
    // Only use the steering callback when in solo mode. Otherwise we'll move based on the required positions to
    // move the arm.
    pnh.param<bool>("solo_mode", soloMode, false);
    if(soloMode){
        ROS_INFO("Running solo mode");
    }
    else {
      ROS_INFO("Running regular mode");
      adjustDogClient.waitForServer();
    }

    pnh.param<bool>("no_steering_mode", noSteeringMode, false);
    
    initTimer = nh.createTimer(ros::Duration(DELAY_TIME), &RobotDriver::init, this, true /* One shot */);
    ROS_INFO("Robot driver initialization complete @ %f", ros::Time::now().toSec());
  }
  
  void init(const ros::TimerEvent& event){
    ROS_INFO("Entering delayed init");
    
    MoveArmToBasePositionGoal moveArmToBasePositionGoal;
    utils::sendGoal(&moveArmToBasePositionClient, moveArmToBasePositionGoal, nh, 10.0);
    
    // Move the robot to the initial orientation.
    ROS_INFO("Moving to initial pose");
    MoveRobotGoal moveRobotGoal;
    moveRobotGoal.pose.header.stamp = ros::Time::now();
    moveRobotGoal.pose.header.frame_id = "/base_footprint";
    moveRobotGoal.pose.pose.orientation = tf::createQuaternionMsgFromYaw(boost::math::constants::pi<double>() / 2.0);
    utils::sendGoal(&moveRobotClient, moveRobotGoal, nh);
    
    ROS_INFO("Clearing wheels");
    // Clear the wheels so we can move forward.
    moveRobotGoal.pose.header.stamp = ros::Time::now();
    moveRobotGoal.pose.header.frame_id = "/base_footprint";
    moveRobotGoal.pose.pose.position.x = 0.2;
    moveRobotGoal.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    utils::sendGoal(&moveRobotClient, moveRobotGoal, nh);
    
    startPath(ros::Time::now());
    if(!noSteeringMode){
        driverTimer = nh.createTimer(ros::Duration(MOVE_ROBOT_UPDATE_INTERVAL), &RobotDriver::steeringCallback, this);
    }
    ROS_INFO("Delayed init complete");
  }
  
    void startPath(const ros::Time& currentTime){
        StartPath startPath;
        startPath.request.time = currentTime.toSec();
        ros::ServiceClient startPathClient = nh.serviceClient<StartPath>("/dogsim/start", false);
        startPathClient.call(startPath);
    }

    void avoidingDogCallback(const AvoidingDogConstPtr& avoidDogMsg){
        ROS_DEBUG("Robot driver received avoid dog callback");
        if(avoidDogMsg->avoiding){
            ROS_INFO("Canceling movement as the avoiding flag is set");
            // Cancel all movement
            adjustDogClient.cancelGoal();
            moveRobotClient.cancelGoal();
            moveArmToBasePositionClient.cancelGoal();
        }
        else if(avoidingDog){
            ROS_INFO("Resetting arm as the avoiding flag is not set");
            // Reset the arm
            MoveArmToBasePositionGoal moveArmToBasePositionGoal;
            moveArmToBasePositionClient.sendGoal(moveArmToBasePositionGoal);
        }
        // Set the flag
        avoidingDog = avoidDogMsg->avoiding;
    }
    
  void dogPositionCallback(const DogPositionConstPtr& dogPosition){
    ROS_DEBUG("Received a dog position callback @ %f", ros::Time::now().toSec());
    
    // No actions can be executed while we are avoiding the dog.
    if(avoidingDog){
        return;
    }
    
    bool ended = false;
    bool started = false;
    const geometry_msgs::PointStamped goalCurrent = getDogGoalPosition(ros::Time(ros::Time::now().toSec()), started, ended);
    if(!started || ended){
        // TODO: Could subscribe and unsubscribe later.
        return;
    }
    
    // Only adjust dog position if the last adjustment finished
    if(adjustDogClient.getState() != actionlib::SimpleClientGoalState::ACTIVE){
        AdjustDogPositionGoal adjustGoal;
        adjustGoal.dogPose = dogPosition->pose;
        adjustGoal.goalPosition = goalCurrent;
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
  
  void steeringCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received steering callback @ %f : %f", event.current_real.toSec(), event.current_expected.toSec());

      bool ended = false;
      bool started = false;
      const geometry_msgs::PointStamped dogGoal = getDogGoalPosition(event.current_real + ros::Duration(MOVE_ROBOT_UPDATE_INTERVAL / 2.0), started, ended);

      if(ended){
          ROS_INFO("Walk ended");
          if(moveRobotClient.getState() == actionlib::SimpleClientGoalState::ACTIVE){
              moveRobotClient.cancelGoal();
          }
          driverTimer.stop();
          return;
      }
      
      const geometry_msgs::PointStamped goal2 = getDogGoalPosition(ros::Time(event.current_real.toSec() + MOVE_ROBOT_UPDATE_INTERVAL / 2.0 + SLOPE_DELTA), started, ended);
      
      // Calculate the vector of the tangent line.
      btVector3 tangent = btVector3(goal2.point.x, goal2.point.y, 0) - btVector3(dogGoal.point.x, dogGoal.point.y, 0);
      tangent.normalize();
 
      // Now select a point on the vector but slightly behind.
      btVector3 backGoal = btVector3(dogGoal.point.x, dogGoal.point.y, 0) - tangent * btScalar(TRAILING_DISTANCE);
   
      // Rotate the vector to perpendicular
      btVector3 perp = tangent.rotate(btVector3(0, 0, 1), btScalar(boost::math::constants::pi<double>() / 2.0));

      // Select a point on the perpindicular line.
      btVector3 finalGoal = backGoal + perp * btScalar(SHIFT_DISTANCE);
  
      geometry_msgs::Point robotGoal;
      robotGoal.x = finalGoal.x();
      robotGoal.y = finalGoal.y();
      robotGoal.z = finalGoal.z();

      MoveRobotGoal moveRobotGoal;
      moveRobotGoal.pose.header = dogGoal.header;
      moveRobotGoal.pose.pose.position = robotGoal;
      
      // Calculate the yaw so we can create an orientation.
      btScalar yaw = btAtan2(tangent.y(), tangent.x());
      moveRobotGoal.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw); 
      ROS_DEBUG("Sending move goal with yaw: %f", yaw);
      
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
