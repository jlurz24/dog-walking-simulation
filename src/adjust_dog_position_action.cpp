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
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

// Generated messages
#include <dogsim/AdjustDogPositionAction.h>

namespace {
  using namespace std;
  using namespace geometry_msgs;
  
  typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveArmClient;

  class AdjustDogPositionAction {
    public:
      AdjustDogPositionAction(const string& name): as(nh, name, boost::bind(&AdjustDogPositionAction::adjust, this, _1), false), actionName(name),
        rightArm("right_arm"){
        
        as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
        
        nh.param("leash_length", leashLength, 2.0);

        handStartPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/hand_start_viz", 1);

        // Setup moveit.
        robot_model_loader::RobotModelLoader robotModelLoader("robot_description", true);
        robot_model::RobotModelPtr kinematicModel = robotModelLoader.getModel();
        modelFrame = kinematicModel->getModelFrame();
        
        kinematicState.reset(new robot_state::RobotState(kinematicModel));
        kinematicState->enforceBounds();
        ROS_DEBUG("Completed init of the adjust dog position action");
        as.start();
  }
  
  void preemptCB(){
    ROS_DEBUG("Preempting the adjust dog position action");

    if(!as.isActive()){
      ROS_DEBUG("Adjust dog position action cancelled prior to start");
      return;
    }

    rightArm.stop();
    as.setPreempted();
  }

  // TODO: This need to handle errors.
  // TODO: Refactor this.
  PointStamped calculateStart(const PointStamped goal, const PoseStamped dogPose){
      
    // Transform the goal position
    PointStamped goalInBaseFrame;
    if(goal.header.frame_id != "/base_footprint"){
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), goal, goal.header.frame_id, goalInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform goal point to /base_footprint");
            as.setAborted();
            return PointStamped();
        }
    }
    else {
        goalInBaseFrame = goal;
    }

    // Transform the dog position.
    PoseStamped dogInBaseFrame;
    if(dogPose.header.frame_id != "/base_footprint"){
        try {
            tf.transformPose("/base_footprint", ros::Time(0), dogPose, dogPose.header.frame_id, dogInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform dog pose to /base_footprint");
            as.setAborted();
            return PointStamped();
        }
    }
    else {
        dogInBaseFrame = dogPose;
    }
    
    // Determine the position of the base in the hand frame
    PointStamped handInBaseFrame;
    {
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "/r_wrist_roll_link";
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame, handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform hand position to /base_footprint");
            as.setAborted();
            return PointStamped();
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
    PointStamped start;
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
    
    PointStamped handStart = calculateStart(goal->goalPosition, goal->dogPose);
    if(handStartPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA YELLOW = utils::createColor(1, 1, 0);
      PointStamped startInBaseFrameViz = handStart;
      visualization_msgs::Marker startMsg = utils::createMarker(startInBaseFrameViz.point, startInBaseFrameViz.header, YELLOW, false);
      handStartPub.publish(startMsg);
    }
      
    // Check if we are still active
    if(!as.isActive()){
        return;
    }
    
    // The caller should abort the movement if it takes too long.
    moveRightArm(handStart);
    as.setSucceeded();
  }

  bool moveRightArm(const PointStamped goalPoint){
     ROS_DEBUG("Moving arm to position %f %f %f in frame %s @ %f", goalPoint.point.x, goalPoint.point.y, goalPoint.point.z, goalPoint.header.frame_id.c_str(), ros::Time::now().toSec());
     
     // Transform to the correct frame.
     PointStamped goalInModelFrame;
     if(goalPoint.header.frame_id != modelFrame){
        try {
            tf.transformPoint(modelFrame, ros::Time(0), goalPoint, goalPoint.header.frame_id, goalInModelFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to goal to %s", modelFrame.c_str());
            return false;
        }
    }
    else {
        goalInModelFrame = goalPoint;
    }
     
     robot_state::JointStateGroup* jointStateGroup = kinematicState->getJointStateGroup("right_arm");
     
     Pose poseGoal;
     poseGoal.position = goalInModelFrame.point;
     poseGoal.orientation.x = poseGoal.orientation.y = poseGoal.orientation.z = 0;
     poseGoal.orientation.z = 1;
     
     kinematics::KinematicsQueryOptions opts;
     bool foundIK = jointStateGroup->setFromIK(poseGoal, 10, 0.1, opts);
     if(foundIK){
         rightArm.setJointValueTarget(*jointStateGroup);
     }
     else {
         ROS_INFO("Failed to find IK solution");
     }
     return true;
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
    string actionName;

    move_group_interface::MoveGroup rightArm;
    robot_state::RobotStatePtr kinematicState;
    
    tf::TransformListener tf;
    
    /**
     * Frame the arm planner uses
     */
    string modelFrame;
    
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
