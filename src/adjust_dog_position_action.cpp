#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <arm_navigation_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/kinematic_constraints/utils.h>

// Generated messages
#include <dogsim/AdjustDogPositionAction.h>

namespace {
  using namespace std;

  typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveArmClient;

  class AdjustDogPositionAction {
    public:
      AdjustDogPositionAction(const string& name): as(nh, name, boost::bind(&AdjustDogPositionAction::adjust, this, _1), false), actionName(name),
        rightArm("move_group", true){
    
        rightArm.waitForServer();
        
        as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
        
        nh.param("leash_length", leashLength, 2.0);

        handStartPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/hand_start_viz", 1);

        ROS_DEBUG("Completed init of the adjust dog position action");
        as.start();
  }
  
  void preemptCB(){
    ROS_DEBUG("Preempting the adjust dog position action");

    if(!as.isActive()){
      ROS_DEBUG("Adjust dog position action cancelled prior to start");
      return;
    }

    if(rightArm.getState() == actionlib::SimpleClientGoalState::ACTIVE){
        ROS_INFO("Preempting the move arm action externally");
        rightArm.cancelGoal();
    }
    as.setPreempted();
  }

  // TODO: This need to handle errors.
  // TODO: Refactor this.
  geometry_msgs::PointStamped calculateStart(const geometry_msgs::PointStamped goal, const geometry_msgs::PoseStamped dogPose){
      
    // Transform the goal position
    geometry_msgs::PointStamped goalInBaseFrame;
    if(goal.header.frame_id != "/base_footprint"){
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), goal, goal.header.frame_id, goalInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform goal point to /base_footprint");
            as.setAborted();
            return geometry_msgs::PointStamped();
        }
    }
    else {
        goalInBaseFrame = goal;
    }

    // Transform the dog position.
    geometry_msgs::PoseStamped dogInBaseFrame;
    if(dogPose.header.frame_id != "/base_footprint"){
        try {
            tf.transformPose("/base_footprint", ros::Time(0), dogPose, dogPose.header.frame_id, dogInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform dog pose to /base_footprint");
            as.setAborted();
            return geometry_msgs::PointStamped();
        }
    }
    else {
        dogInBaseFrame = dogPose;
    }
    
    // Determine the position of the base in the hand frame
    geometry_msgs::PointStamped handInBaseFrame;
    {
        geometry_msgs::PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "/r_wrist_roll_link";
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame, handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform hand position to /base_footprint");
            as.setAborted();
            return geometry_msgs::PointStamped();
        }
     }
    
    // Givens: Dog position + goal position
    // create a line between the dog position and goal position
    // select a point on that line that is leash distance from the dog towards
    // goal.
    // TODO: Allow other points on the arc to be acceptable.
    // TODO: Select random acceptable points on z axis.
    
    // Determine the horizontal position assuming the desired height of the arm is 1.
    double armHeight = handInBaseFrame.point.z; // Height of the robot hand relative to base (approximately the height of the dog).
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
    
    double distanceFromDogToGoal = utils::pointToPointXYDistance(goalInBaseFrame.point, dogInBaseFrame.pose.position);
    ROS_DEBUG("Distance from dog to goal %f", distanceFromDogToGoal);
    
    // Now update the goal to move to the dog to the goal point.
    geometry_msgs::PointStamped start;
    start.point.x = dogInBaseFrame.pose.position.x + (planarLeashLength + distanceFromDogToGoal) * ux;
    start.point.y = dogInBaseFrame.pose.position.x + (planarLeashLength + distanceFromDogToGoal) * uy;
    start.point.z = armHeight;
    start.header = dogInBaseFrame.header;
    return start;
  }
  
  void adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    ROS_DEBUG("Adjusting dog position");
      
    if(!as.isActive()){
        ROS_INFO("Adjust dog position action cancelled prior to start");
        return;
    }
    
    geometry_msgs::PointStamped handStart = calculateStart(goal->goalPosition, goal->dogPose);
    if(handStartPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA YELLOW = utils::createColor(1, 1, 0);
      geometry_msgs::PointStamped startInBaseFrameViz = handStart;
      visualization_msgs::Marker startMsg = utils::createMarker(startInBaseFrameViz.point, startInBaseFrameViz.header, YELLOW, false);
      handStartPub.publish(startMsg);
    }
      
    // Check if we are still active
    if(!as.isActive()){
        return;
    }
    
    // The caller should abort the movement if it takes too long.
    moveRightArm(handStart);
    rightArm.waitForResult(ros::Duration(1.0));
    
    if(rightArm.getState() == actionlib::SimpleClientGoalState::ACTIVE){
        ROS_INFO("Preempting the move arm action inside AdjustDogPosition");
        rightArm.cancelGoal();
    }
    as.setSucceeded();
  }

  bool moveRightArm(const geometry_msgs::PointStamped goalPoint){
     ROS_DEBUG("Moving arm to position %f %f %f in frame %s @ %f", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str(), ros::Time::now().toSec());

     moveit_msgs::MoveGroupGoal goal;
     goal.request.group_name = "right_arm";
     goal.request.num_planning_attempts = 1;
     goal.request.allowed_planning_time = ros::Duration(0.25).toSec();
     goal.request.goal_constraints.resize(1);
     goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", goalPoint, 0.25 /* tolerance */);

     rightArm.sendGoal(goal);
     return true;
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
    string actionName;

    MoveArmClient rightArm;
    
    tf::TransformListener tf;
    
    //! Publisher for hand start position.
    ros::Publisher handStartPub;
    double leashLength;
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "adjust_dog_position_action");
  AdjustDogPositionAction action(ros::this_node::getName());
  ROS_INFO("Waiting for adjust_dog_position actions");
  ros::spin();
  
  return 0;
}
