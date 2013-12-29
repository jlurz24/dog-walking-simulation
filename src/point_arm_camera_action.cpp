#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/btVector3.h>
#include <dogsim/utils.h>

// Generated messages
#include <dogsim/PointArmCameraAction.h>

namespace {
using namespace std;
using namespace ros;

static const string MOVE_GROUP_NAME_DEFAULT = "right_arm";
class PointArmCamera {
private:
    NodeHandle nh;
    NodeHandle pnh;
    actionlib::SimpleActionServer<dogsim::PointArmCameraAction> as;
    string actionName;
    move_group_interface::MoveGroup arm;
    tf::TransformListener tf;

    //! Publisher for the look direction
    ros::Publisher lookDirectionPub;
public:
    PointArmCamera(const string& name) :
            pnh("~"),
            as(nh, name, boost::bind(&PointArmCamera::moveArmToTarget, this, _1), false),
            actionName(
                    name),
                    arm("right_arm") {
        lookDirectionPub = nh.advertise<visualization_msgs::Marker>(
                "/point_arm_camera_action/look_direction_viz", 1);

        as.registerPreemptCallback(boost::bind(&PointArmCamera::preemptCB, this));
        as.start();
    }

protected:
    void preemptCB() {
        ROS_DEBUG("Preempting the point arm camera action");

        if (!as.isActive()) {
            ROS_DEBUG("Point arm camera action cancelled prior to start");
            return;
        }

        arm.stop();
        as.setPreempted();
    }

    bool moveArmToTarget(const dogsim::PointArmCameraGoalConstPtr& goal) {
        if (!as.isActive()) {
            ROS_INFO("Move arm to clear position cancelled prior to start");
            return false;
        }

        ROS_INFO("Moving arm to point at point %f %f %f in frame %s",
                goal->target.point.x, goal->target.point.y, goal->target.point.z, goal->target.header.frame_id.c_str());

        // Transform the goal position to the camera frame
        geometry_msgs::PointStamped goalInCameraFrame;
        if(goal->target.header.frame_id != "/r_forearm_cam_frame"){
            try {
                tf.transformPoint("/r_forearm_cam_frame", ros::Time(0), goal->target, goal->target.header.frame_id, goalInCameraFrame);
            }
            catch(tf::TransformException& ex){
                ROS_INFO("Failed to transform goal point to /r_forearm_cam_frame");
                as.setAborted();
                return false;
            }
        }
        else {
            goalInCameraFrame = goal->target;
        }

        ROS_INFO("Moving arm to point at point %f %f %f in frame %s",
                goalInCameraFrame.point.x, goalInCameraFrame.point.y, goalInCameraFrame.point.z, goalInCameraFrame.header.frame_id.c_str());

        // Now create a unit vector
        btVector3 direction(goalInCameraFrame.point.x, goalInCameraFrame.point.y, goalInCameraFrame.point.z);
        direction.normalize();

        ROS_INFO("Unit vector in camera frame %f %f %f", direction.x(), direction.y(), direction.z());

        btVector3 planeDirection(direction.x(), direction.y(), 0);
        // Compute RPY
        double yaw = atan2(-direction.y(), direction.x());
        double pitch = atan2(direction.z(), planeDirection.length());
        double roll = arm.getCurrentRPY(arm.getEndEffectorLink())[0];

        ROS_INFO("Creating quaternion with RPY: %f %f %f", roll, pitch, yaw);
        geometry_msgs::QuaternionStamped orientation;
        orientation.quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        orientation.header.frame_id = "/r_forearm_cam_optical_frame";
        orientation.header.stamp = goal->target.header.stamp;

        // Convert the pose to the planning frame of the arm.
        geometry_msgs::QuaternionStamped orientationInPlanningFrame;
        ROS_INFO("Converting to frame %s", arm.getPlanningFrame().c_str());
        tf.transformQuaternion(arm.getPlanningFrame(), orientation, orientationInPlanningFrame);

        geometry_msgs::PoseStamped pose;
        pose.header = orientationInPlanningFrame.header;
        pose.pose.orientation = orientationInPlanningFrame.quaternion;
        ROS_INFO("Publishing pose in frame: %s with reference frame: %s", pose.header.frame_id.c_str(), arm.getPoseReferenceFrame().c_str());

        if(lookDirectionPub.getNumSubscribers() > 0){
            visualization_msgs::Marker marker;
            marker.header = pose.header;
            marker.ns = ros::this_node::getName();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose.pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color = utils::createColor(1, 0, 0);
            marker.lifetime = ros::Duration();
            lookDirectionPub.publish(marker);
        }

        arm.setOrientationTarget(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w, "r_forearm_roll_link");
        arm.move();
        as.setSucceeded();
        return true;
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_arm_camera_action");
    PointArmCamera action(ros::this_node::getName());
    ros::spin();
}
