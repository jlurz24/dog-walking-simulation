#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
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

        movePub = nh.advertise<visualization_msgs::Marker>("move_robot_action/planned_move_viz", 1);
        as.start();
  }
  
  protected:
  void preemptCB(){
    ROS_DEBUG("Preempting the move robot action");

    if(!as.isActive()){
      ROS_INFO("Move robot position action canceled prior to start");
      return;
    }

    // Do not stop the robot as this will be called if we are preempted by the sequencer.
    as.setPreempted();
  }

  void stop(){
	ROS_INFO("Stopping the base");
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    
    // Publish the command to the base
    cmdVelocityPub.publish(baseCmd);
  }
  
  static double calcQDistance(const tf::Quaternion& q1, const tf::Quaternion& q2){
      return (1 - btPow(q1.dot(q2), btScalar(2)));
  }
  
  void move(const dogsim::MoveRobotGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Move robot action canceled prior to start");
      return;
    }
    
    // Goals should always be in the map frame
    assert(goal->poses.size() > 0 && goal->poses[0].header.frame_id == "/map");
    
    // Find the goal closest to the current time in the future. Plans are always in
    // increasing order.
    bool found = false;
    geometry_msgs::PoseStamped absoluteGoal;
    ros::Time now = ros::Time::now();
    for(unsigned int i = 0; i < goal->poses.size(); ++i){
        if(goal->poses[i].header.stamp > now){
            absoluteGoal = goal->poses[i];
        }
    }
    if(!found){
        ROS_ERROR("Could not locate any future goals");
        as.setAborted();
        return;
    }

    // Determine the updated position of the robot.
    geometry_msgs::PoseStamped goalPose;
    try {
        tf.transformPose("/base_footprint", ros::Time(0), absoluteGoal, absoluteGoal.header.frame_id, goalPose);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Failed to transform absolute goal from %s to /base_footprint frame", absoluteGoal.header.frame_id.c_str());
        as.setAborted();
        return;
    }
    
    // Loop and move the robot, checking the position of the goal in the base
    // frame after each move.
    static const unsigned int HZ = 50;
    ros::Rate r(HZ);
    
    // How close to the goal we need to get.
    static const double DISTANCE_THRESHOLD = 0.1;
    
    // This is about 2.5 degrees
    static const double Q_DISTANCE_THRESHOLD = 0.005;
    static const double MAX_LINEAR_V = 0.5;
    static const double STRAIGHT_AHEAD_FACTOR = 6.0;
    static const double MAX_ANGULAR_V = 0.5;
    static const double STATIONARY_FACTOR = 4.0;
    static const double MAX_REVERSE_DISTANCE = 0.25;
            
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

    double qDistance = calcQDistance(goalPoseTF, robotPoseTF);
    
    // Loop and move the robot until the robot reaches the goal point.
    while(ros::ok() && as.isActive() && (currentDistance > DISTANCE_THRESHOLD || qDistance > Q_DISTANCE_THRESHOLD)){
        ROS_DEBUG("Distance to goal: %f %f", currentDistance, qDistance);
        
        // Attempt to estimate the distance traveled in this iteration
        double ratio = 1;
        
        // Use the ratio of current distance to go over the total distance to go assuming the total
        // distance is not zero.
        if(totalDistance > DISTANCE_THRESHOLD){
            ratio = min((totalDistance - (currentDistance - MAX_LINEAR_V * 1.0 / static_cast<double>(HZ))) / totalDistance, 1.0);
        }
        ROS_DEBUG("Total distance %f current distance %f ratio %f", totalDistance, currentDistance, ratio);
        
        // Interpolate between the starting orientation and goal.
        btVector3 goalVector(goalPose.pose.position.x, goalPose.pose.position.y, 0);
        btScalar yawToTarget = btAtan2(goalVector.y(), goalVector.x());
        tf::Quaternion goalOrientation = tf::createQuaternionFromYaw(yawToTarget);
        
        tf::Quaternion interp = tf::slerp(goalOrientation, goalPoseTF, ratio);
        double yaw = tf::getYaw(interp);
        
        // Calculate the movement.
        geometry_msgs::Twist baseCmd;


        
        // Robot location is at the root of frame.
        if(currentDistance > DISTANCE_THRESHOLD){
            baseCmd.linear.x = MAX_LINEAR_V;
        }
        else {
            baseCmd.linear.x = 0;
        }

        ROS_DEBUG("Final yaw: %f x: %f qdistance: %f", yaw, baseCmd.linear.x, qDistance);
        
        // Determine if we should go backwards.
        if(goalPose.pose.position.x < 0 && currentDistance > DISTANCE_THRESHOLD && fabs(yaw > 3 * PI / 4)){
            if(currentDistance < MAX_REVERSE_DISTANCE){
                ROS_INFO("Moving in reverse at original anglular velocity %f at velocity %f", yaw, baseCmd.linear.x);
                if(yaw > PI / 2.0){
                    yaw -= PI;
                } else {
                    yaw += PI;
                }
                baseCmd.linear.x *= -1;
                ROS_INFO("Moving in reverse at anglular velocity %f at velocity %f", yaw, baseCmd.linear.x);
            }
            else {
                // Use yaw as is.
                baseCmd.linear.x = 0;
            }
        }
        else if(calcQDistance(tf::createIdentityQuaternion(), interp) < Q_DISTANCE_THRESHOLD){
            ROS_DEBUG("Not performing turn because threshold reached. yaw = %f qDistance = %f", yaw, calcQDistance(tf::createIdentityQuaternion(), interp));
            // Turning slows the robot down, so avoid turning when possible.
            yaw = 0;
        }
        // Turn and drive
        else {
            ROS_DEBUG("Turning and driving. Yaw = %f qDistance = %f", yaw, calcQDistance(tf::createIdentityQuaternion(), interp));
            // Use yaw as is.     
        }
        
        // base cmd is rotational
        if(fabs(yaw) > numeric_limits<double>::epsilon()){
            baseCmd.angular.z = copysign(MAX_ANGULAR_V, yaw);
            if(fabs(baseCmd.linear.x) < numeric_limits<double>::epsilon()){
                // If the robot is not moving forward use a faster turn.
                baseCmd.angular.z *= STATIONARY_FACTOR;
            }
            else {
                ROS_DEBUG("X too high to apply stationary factor %f", baseCmd.linear.x);
            }
        }
        else {
            baseCmd.linear.x *= STRAIGHT_AHEAD_FACTOR;
            baseCmd.angular.z = 0;
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
                ROS_ERROR("Failed to transform absolute goal from %s to /base_footprint frame", absoluteGoal.header.frame_id.c_str());
                // Continue and hope that it transforms next time.
            }
            currentDistance = utils::pointToPointXYDistance(goalPose.pose.position, robotPose.position);
            tf::quaternionMsgToTF(goalPose.pose.orientation, goalPoseTF);
            qDistance = calcQDistance(goalPoseTF, robotPoseTF);
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

  private:
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
