#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <visualization_msgs/Marker.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/btVector3.h>
#include <moveit_msgs/GetPositionIK.h>
#include <boost/bind.hpp>

// Generated messages
#include <dogsim/AdjustDogPositionAction.h>

namespace {
  using namespace std;
  using namespace geometry_msgs;

  class AdjustDogPositionAction {
    public:
      AdjustDogPositionAction(const string& name): as(nh, name, boost::bind(&AdjustDogPositionAction::adjust, this, _1), false), actionName(name),
        rightArm("right_arm"){
        
        as.registerPreemptCallback(boost::bind(&AdjustDogPositionAction::preemptCB, this));
        
        nh.param("leash_length", leashLength, 2.0);

        handStartPub = nh.advertise<visualization_msgs::Marker>("adjust_dog_position_action/hand_start_viz", 1);

        // Setup moveit.
        robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
        robot_model::RobotModelPtr kinematicModel = robotModelLoader.getModel();
        
        kinematicState.reset(new robot_state::RobotState(kinematicModel));
        
        ros::service::waitForService("compute_ik");
        ikClient = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
        
        ROS_INFO("Completed init of the adjust dog position action");
        as.start();
    }
  
    private:
  
    void preemptCB(){
        ROS_DEBUG("Preempting the adjust dog position action");

        if(!as.isActive()){
            ROS_DEBUG("Adjust dog position action canceled prior to start");
            return;
        }

        rightArm.stop();
        as.setPreempted();
    }

    PointStamped calculateStart(const PointStamped goalInBaseFrame, const PoseStamped dogInBaseFrame, const double armHeight, const double leashLength, const double angle) const {
    
        ROS_DEBUG("Searching for solution at arm height: %f for leash length: %f", armHeight, leashLength);
    
        double planarLeashLength = 0;
        if(leashLength - armHeight > 0){
            planarLeashLength = sqrt(utils::square(leashLength) - utils::square(armHeight));
        }
        ROS_DEBUG("Arm height: %f planar leash length: %f", armHeight, planarLeashLength);

        // Find the angle between the dog and the goal point.
        // Calculate the unit vector given x1, y1 = dog and x2, y2 = goal
        btVector3 dogToGoal = btVector3(goalInBaseFrame.point.x, goalInBaseFrame.point.y, armHeight) - btVector3(dogInBaseFrame.pose.position.x, dogInBaseFrame.pose.position.y, armHeight);
        dogToGoal.normalize();
    
        double distanceFromDogToGoal = utils::pointToPointXYDistance(goalInBaseFrame.point, dogInBaseFrame.pose.position);
        ROS_DEBUG("Distance from dog to goal %f", distanceFromDogToGoal);
    
        btVector3 dogVector(dogInBaseFrame.pose.position.x, dogInBaseFrame.pose.position.y, armHeight);
        btVector3 startVector = dogVector + btScalar(distanceFromDogToGoal) * dogToGoal;
        
        // Now add the leash length.
        btVector3 leashToGoal = dogToGoal.rotate(btVector3(0, 0, 1), btScalar(angle));
        startVector = startVector + btScalar(planarLeashLength) * leashToGoal;
        
        // Now update the goal to move to the dog to the goal point.
        PointStamped start;
        start.point.x = startVector.x();
        start.point.y = startVector.y();
        start.point.z = startVector.z();
        start.header = dogInBaseFrame.header;
        return start;
  }
  
  bool adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    ROS_DEBUG("Adjusting dog position");
      
    if(!as.isActive()){
        ROS_INFO("Adjust dog position action cancelled prior to start");
        return false;
    }
    
    // Transform the goal position
    PointStamped goalInBaseFrame;
    if(goal->goalPosition.header.frame_id != "/base_footprint"){
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), goal->goalPosition,goal->goalPosition.header.frame_id, goalInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform goal point to /base_footprint");
            as.setAborted();
            return false;
        }
    }
    else {
        goalInBaseFrame = goal->goalPosition;
    }

    // Transform the dog position.
    PoseStamped dogInBaseFrame;
    if(goal->dogPose.header.frame_id != "/base_footprint"){
        try {
            tf.transformPose("/base_footprint", ros::Time(0), goal->dogPose, goal->dogPose.header.frame_id, dogInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Failed to transform dog pose from %s to /base_footprint frame", goal->dogPose.header.frame_id.c_str());
            as.setAborted();
            return false;
        }
    }
    else {
        dogInBaseFrame = goal->dogPose;
    }
    
    // Determine the position of the base in the hand frame
    PointStamped handInBaseFrame;
    {
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = rightArm.getEndEffectorLink();
        ROS_DEBUG("End effector link is %s", rightArm.getEndEffectorLink().c_str());
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame, handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform hand position to /base_footprint");
            as.setAborted();
            return false;
        }
     }
    
    // Lookup the transform between the base plan and the planning frame so we can cache it.
    tf::StampedTransform baseToPlanningTransform;
    tf.lookupTransform(rightArm.getPlanningFrame(), "/base_footprint", ros::Time(0), baseToPlanningTransform);
    
    // Givens: Dog position + goal position
    // create a line between the dog position and goal position
    // select a point on that line that is leash distance from the dog towards
    // goal.
    // TODO: Allow other points on the arc to be acceptable.
    
    // Determine the current vertical position of the arm.
    double armHeight = handInBaseFrame.point.z; // Height of the robot hand relative to base (approximately the height of the dog).
    ros::Time startTime = ros::Time::now();
    
    // First try at the current height
    // TODO: Refactor
    PointStamped handStart = calculateStart(goalInBaseFrame, dogInBaseFrame, armHeight, leashLength, 0);
        
    // The caller should abort the movement if it takes too long.
    vector<double> positions;
    bool found = moveRightArm(applyTransform(handStart, baseToPlanningTransform), positions);
    
    static const double angles[5] = { 0.0,
                                      boost::math::constants::pi<double>() / 8.0,
                                     -boost::math::constants::pi<double>() / 8.0,
                                      boost::math::constants::pi<double>() / 4.0,
                                     -boost::math::constants::pi<double>() / 4.0
                                     };
    
    if(!found && as.isActive()){
        // Iterate over angles
        for(unsigned int i = 0; i < 3 && !found && as.isActive(); ++i){
            // Iterate over all possible height
            for(double height = MIN_ARM_HEIGHT; !found && as.isActive() && height <= min(leashLength, MAX_ARM_HEIGHT); height += ARM_HEIGHT_SEARCH_INCREMENT){
                handStart = calculateStart(goalInBaseFrame, dogInBaseFrame, height, leashLength, angles[i]);
        
                if(handStartPub.getNumSubscribers() > 0){
                    static const std_msgs::ColorRGBA YELLOW = utils::createColor(1, 1, 0);
                    PointStamped startInBaseFrameViz = handStart;
                    visualization_msgs::Marker startMsg = utils::createMarker(startInBaseFrameViz.point, startInBaseFrameViz.header, YELLOW, false);
                    handStartPub.publish(startMsg);
                }
                
                found = moveRightArm(applyTransform(handStart, baseToPlanningTransform), positions);
            }
        }
    }
    
   if(found){
     ROS_DEBUG("IK succeeded in %f", ros::Time::now().toSec() - startTime.toSec());
     ros::Time planStartTime = ros::Time::now();
     rightArm.setJointValueTarget(positions);
     moveit::planning_interface::MoveGroup::Plan plan;
     bool success = rightArm.plan(plan);
     ROS_DEBUG("Planning completed in %f", ros::Time::now().toSec() - planStartTime.toSec());
     if(success){
        rightArm.execute(plan);
     }
     else {
       ROS_INFO("Planning failed after %f", ros::Time::now().toSec() - planStartTime.toSec());
     }
   }
   else {
        ROS_INFO("Failed to find solution after %f", ros::Time::now().toSec() - startTime.toSec());
    }

    if(found && handStartPub.getNumSubscribers() > 0){
      static const std_msgs::ColorRGBA YELLOW = utils::createColor(1, 1, 0);
      PointStamped startInBaseFrameViz = handStart;
      visualization_msgs::Marker startMsg = utils::createMarker(startInBaseFrameViz.point, startInBaseFrameViz.header, YELLOW, false);
      handStartPub.publish(startMsg);
    }
    
    as.setSucceeded();
    return true;
  }

  static Point applyTransform(const geometry_msgs::PointStamped& point, const tf::StampedTransform transform){
    tf::Stamped<tf::Point> tfPoint;
    tf::pointStampedMsgToTF(point, tfPoint);
    tf::Point tfGoal = transform * tfPoint;
    Point msgGoal;
    tf::pointTFToMsg(tfGoal, msgGoal);
    return msgGoal;
  }
  
  bool moveRightArm(const Point goalPoint, vector<double>& positions){
     // Transform to the planning frame.
     ROS_DEBUG("Moving arm to position %f %f %f in frame %s @ %f", goalPoint.x, goalPoint.y, goalPoint.z, rightArm.getPlanningFrame().c_str(), ros::Time::now().toSec());
     
     moveit_msgs::GetPositionIK::Request req;
     moveit_msgs::GetPositionIK::Response res; 
     req.ik_request.group_name = "right_arm";
     
     // req.ik_request.constraints = kinematic_constraints::constructGoalConstraints("r_gripper_l_finger_tip_link", point, 5);
     req.ik_request.pose_stamped.header.frame_id = rightArm.getPlanningFrame();
     req.ik_request.pose_stamped.pose.position = goalPoint;
     req.ik_request.pose_stamped.pose.orientation.x = 0.0;
     req.ik_request.pose_stamped.pose.orientation.y = 0.0;
     req.ik_request.pose_stamped.pose.orientation.z = 0.0;
     req.ik_request.pose_stamped.pose.orientation.w = 1.0;
     
     req.ik_request.avoid_collisions = false;
     // req.ik_request.attempts = 10;
     // Default is 0.05
     req.ik_request.timeout = ros::Duration(0.05);
     
     const robot_state::JointStateGroup* jointStateGroup = kinematicState->getJointStateGroup("right_arm");
     const vector<string>& jointNames = jointStateGroup->getJointModelGroup()->getJointModelNames();
     
     // Seed state defaults to current positions
     
     ikClient.call(req, res);
     if(res.error_code.val == res.error_code.SUCCESS){
         
         // For some reason this returns all joints. Copy over ones we need.
         positions.resize(jointNames.size());
         for(unsigned int i = 0; i < jointNames.size(); ++i){
             for(unsigned int j = 0; j < res.solution.joint_state.name.size(); ++j){
                 if(jointNames[i] == res.solution.joint_state.name[j]){
                     positions[i] = res.solution.joint_state.position[j];
                     break;
                 }
             }
         }
     }
     else {
         ROS_DEBUG_STREAM("Failed to find IK solution. Error code " << res.error_code);
         return false;
     }
     return true;
  }
  
    protected:
    
        // Calibrated through experimentation.
        static const double MAX_ARM_HEIGHT = 1.376;
        
        // It is possible to get lower than this height, but likely will not help for optimization.
        static const double MIN_ARM_HEIGHT = 0.2;
        
        static const double ARM_HEIGHT_SEARCH_INCREMENT = 0.1;
    
        ros::NodeHandle nh;
    
        // Actionlib classes
        actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
        string actionName;

        move_group_interface::MoveGroup rightArm;
        robot_state::RobotStatePtr kinematicState;
        ros::ServiceClient ikClient;
        
        tf::TransformListener tf;
    
        //! Publisher for hand start position.
        ros::Publisher handStartPub;
    
        //! Length of the leash
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
