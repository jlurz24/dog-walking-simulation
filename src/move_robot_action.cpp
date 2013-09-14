#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/btVector3.h>

// Generated messages
#include <dogsim/MoveRobotAction.h>

namespace {
  using namespace std;

  static const double PI = boost::math::constants::pi<double>();
  
  class MoveRobotAction {
    public:
      MoveRobotAction(const string& name): as(nh, name, boost::bind(&MoveRobotAction::move, this, _1), false), actionName(name) {
        as.registerPreemptCallback(boost::bind(&MoveRobotAction::preemptCB, this));
      
        // Set up the publisher for the cmd_vel topic
        cmdVelocityPub = nh.advertise<geometry_msgs::Twist>("base_controller/command", 1, true);

        goalPub = nh.advertise<visualization_msgs::Marker>("move_robot_action/move_goal_viz", 1);
        movePub = nh.advertise<visualization_msgs::Marker>("move_robot_action/planned_move_viz", 1);
        as.start();
  }
  
  void preemptCB(){
    ROS_DEBUG("Preempting the move robot action");

    if(!as.isActive()){
      ROS_INFO("Move robot position action cancelled prior to start");
      return;
    }
    // Stop the robot.
    stop();
    as.setPreempted();
  }

  void stop(){
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    
    // Publish the command to the base
    cmdVelocityPub.publish(baseCmd);
  }
  
  void move(const dogsim::MoveRobotGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Move robot action cancelled prior to start");
      return;
    }
    
    // Visualize the goal.
    if(goalPub.getNumSubscribers()){
        std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
        goalPub.publish(utils::createMarker(goal->pose.pose.position, goal->pose.header, GREEN, false));
    }
    
    // Determine the pose in the map frame. This allows us to store
    // an absolute pose
    geometry_msgs::PoseStamped absoluteGoal;
    try {
        tf.transformPose("/map", ros::Time(0), goal->pose, goal->pose.header.frame_id, absoluteGoal);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Failed to transform goal to map frame");
        as.setAborted();
        return;
    }
    
    // Determine the updated position of the robot.
    geometry_msgs::PoseStamped goalPose;
    try {
        tf.transformPose("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPose);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Failed to transform absolute goal to base footprint");
        as.setAborted();
        return;
    }
    
    // Loop and move the robot, checking the position of the goal in the base
    // frame after each move.
    ros::Rate r(10); // 10hz
    
    // How close to the goal we need to get.
    const double DISTANCE_THRESHOLD = 0.1;
    const double Q_DISTANCE_THRESHOLD = 0.01;
    
    // Robot is at 0,0
    geometry_msgs::Pose robotPose;
    tf::quaternionTFToMsg(tf::createIdentityQuaternion(), robotPose.orientation);
    
    // Track the total distance.
    double totalDistance = utils::pointToPointXYDistance(goalPose.pose.position, robotPose.position);
    double currentDistance = totalDistance;
    
    // Convert to bt classes
    tf::Quaternion robotPoseTF;
    tf::quaternionMsgToTF(robotPose.orientation, robotPoseTF);
    
    tf::Quaternion goalPoseTF;
    tf::quaternionMsgToTF(goalPose.pose.orientation, goalPoseTF);

    double qDistance = 1 - btPow(goalPoseTF.dot(robotPoseTF), btScalar(2));
    
    // Loop and move the robot until the robot reaches the goal point.
    while(ros::ok() && as.isActive() && (currentDistance > DISTANCE_THRESHOLD || qDistance > Q_DISTANCE_THRESHOLD)){
        ROS_DEBUG("Distance to goal: %f %f", currentDistance, qDistance);
        
        // Interpolate between the starting orientation and goal.
        const double MAX_V = 2.5;
        
        // Attempt to estimate the distance traveled in this iteration
        double ratio = 1;
        
        // TODO: Better small number constant
        if(totalDistance > 0.01){
            ratio = min(max(totalDistance - currentDistance, MAX_V * 0.1) / totalDistance, 1.0);
        }
        
        btVector3 goalVector(goalPose.pose.position.x, goalPose.pose.position.y, 0);
        btScalar yawToTarget = btAtan2(goalVector.y(), goalVector.x());
        tf::Quaternion goalOrientation = tf::createQuaternionFromYaw(yawToTarget);
        
        ROS_DEBUG("Orientation ratio: %f", ratio);
        tf::Quaternion interp = tf::slerp(goalOrientation, goalPoseTF, ratio);
        double yaw = tf::getYaw(interp);
        
        // Calculate the movement.
        geometry_msgs::Twist baseCmd;

        const double DEACC_DISTANCE = 0.75;
        const double MAX_REVERSE_DISTANCE = 0.5;
        
        // Robot location is at the root of frame.
        if(currentDistance > DEACC_DISTANCE){
          baseCmd.linear.x = MAX_V;
        }
        else {
          baseCmd.linear.x = currentDistance / DEACC_DISTANCE * MAX_V;
        }

        ROS_DEBUG("Final yaw: %f x: %f", yaw, baseCmd.linear.x);
        
        // Determine if we should go backwards.
        // TODO: Might need to tune this further to perform the least amount of turning.
        if(goalPose.pose.position.x < 0){
            if(goalPose.pose.position.x > -MAX_REVERSE_DISTANCE && goalPose.pose.position.x > 0){
                if(yaw > PI){
                    baseCmd.angular.z = yaw - PI;
                } else {
                    baseCmd.angular.z = yaw + PI;
                }
                baseCmd.linear.x *= -1;
                ROS_INFO("Moving in reverse at angle %f at velocity %f", baseCmd.angular.z, baseCmd.linear.x);
            }
            else {
                baseCmd.angular.z = yaw;
                baseCmd.linear.x = 0;
            }
        }
        else if(fabs(yaw) > PI / 4.0){
            ROS_DEBUG("Too far to turn and move: %f", yaw);
            baseCmd.linear.x = 0;
            baseCmd.angular.z = yaw;
        }
        else {
            baseCmd.angular.z = yaw;      
        }
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
                tf.transformPose("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPose);
            }
            catch(tf::TransformException& ex){
                ROS_ERROR("Failed to transform absolute goal to base footprint");
                // Continue and hope that it transforms next time.
            }
            currentDistance = utils::pointToPointXYDistance(goalPose.pose.position, robotPose.position);
            tf::quaternionMsgToTF(goalPose.pose.orientation, goalPoseTF);
            qDistance = 1 - btPow(goalPoseTF.dot(robotPoseTF), btScalar(2));
        }
        
    }
    
    if(as.isActive()){
        stop();
        ROS_DEBUG("Move robot completed. Goal achieved.");
        as.setSucceeded();
    }
    else {
        ROS_DEBUG("Move aborted prior to completion");
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
  ros::init(argc, argv, "move_robot_action");
  MoveRobotAction action(ros::this_node::getName());
  ros::spin();
  return 0;
}
