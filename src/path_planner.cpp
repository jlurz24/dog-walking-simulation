#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <dogsim/NextGoal.h>
#include <dogsim/GetEntireRobotPath.h>

namespace {
using namespace std;
using namespace dogsim;

class PathPlanner {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

	//! Transform listener
	tf::TransformListener tf;

	//! Publisher for the current plan
	ros::Publisher planPublisher;

    /*
     * Costmap for the planner
     */
    costmap_2d::Costmap2DROS costmap;

    /*
     * Local planner
     */
    dwa_local_planner::DWAPlannerROS tp;

    bool running;

    //! Cached service client.
    ros::ServiceClient getEntireRobotPathClient;

public:
	PathPlanner() :
		pnh("~"), tf(ros::Duration(10), true), costmap("local_costmap", tf) {

        ros::service::waitForService("/dogsim/get_entire_robot_path");

        getEntireRobotPathClient = nh.serviceClient<GetEntireRobotPath>("/dogsim/get_entire_robot_path", true /* persist */);

        ros::SubscriberStatusCallback connectCB = boost::bind(&PathPlanner::startListening,
                this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &PathPlanner::stopListening, this);
		planPublisher = nh.advertise<NextGoal>("/robot_path/next_goal", 1, connectCB, disconnectCB);
		costmap.start();
        tp.initialize("local_planner", &tf, &costmap);
        costmap.pause();
		running = false;
	}

    void stopListening() {
        if (planPublisher.getNumSubscribers() == 0) {
            ROS_DEBUG("Stopping path planner");
            costmap.pause();
            running = false;
        }
    }

    void startListening() {
        if (planPublisher.getNumSubscribers() == 1) {
            ROS_INFO("Starting path planner");
            costmap.resume();
            running = true;
            publishCurrentPlan();
        }
    }

    bool publishCurrentPlan(){
        assert(tp.isInitialized());

        GetEntireRobotPath getPath;
        getPath.request.increment = 0.25;
        getEntireRobotPathClient.call(getPath);
        ROS_INFO("Path has %lu poses", getPath.response.poses.size());

        if (getPath.response.poses.size() == 0) {
            ROS_ERROR("No goal poses sent to local planner");
            return false;
        }

        if (getPath.response.poses[0].header.frame_id != costmap.getGlobalFrameID()) {
            ROS_ERROR("Goal frame %s must match costmap frame %s",
                    getPath.response.poses[0].header.frame_id.c_str(), costmap.getGlobalFrameID().c_str());
            return false;
        }


        // Loop over the path and select the next target at each callback
        ros::Duration pathUpdateRate(2, 0);

        // How far to look ahead.
        ros::Duration pathLookahead(2, 0);

        // Loop and move the robot, checking the position of the goal in the base
        // frame after each move.
        ros::Rate updateRate(10);

        while (ros::ok() && running) {
            tf::Stamped<tf::Pose> globalPose;
            if (!costmap.getRobotPose(globalPose)) {
                ROS_ERROR(
                        "Cannot make a plan because the local planner could not get the start pose of the robot");
                continue;
            }

            const ros::Time nextUpdateTime = ros::Time::now() + pathUpdateRate;

            ROS_DEBUG("Recomputing path segment");
            vector<geometry_msgs::PoseStamped>::const_iterator i;

            // We always want to include the last pose if there are no
            // future poses.
            for (i = getPath.response.poses.begin(); i != getPath.response.poses.end() - 1; ++i) {
                if (i->header.stamp > ros::Time::now()) {
                    break;
                }
            }

            vector<geometry_msgs::PoseStamped>::const_iterator j;
            for (j = i; j != getPath.response.poses.end(); ++j) {
                if (j->header.stamp > ros::Time::now() + pathLookahead) {
                    break;
                }
            }

            // Create a list of all future poses.
            const vector<geometry_msgs::PoseStamped> currentPath = vector<geometry_msgs::PoseStamped>(i, j);
            assert(currentPath.size() > 0 && "No poses in current path");

            ROS_DEBUG("Setting plan for local planner");
            if (!tp.setPlan(currentPath)) {
                ROS_ERROR("Failed to set plan");
                continue;
            }

            while (running && ros::ok() && !tp.isGoalReached() && ros::Time::now() < nextUpdateTime) {

                geometry_msgs::Twist baseCmd;
                if (!tp.computeVelocityCommands(baseCmd)) {
                    ROS_WARN("Failed to compute velocity commands. Publishing zero velocity.");
                    NextGoal goal;
                    goal.twist.linear.x = 0.0;
                    goal.twist.linear.y = 0.0;
                    goal.twist.angular.z = 0.0;
                    planPublisher.publish(goal);
                }
                else {
                    ROS_DEBUG("Successfully computed base command %f %f %f", baseCmd.linear.x,
                            baseCmd.linear.y, baseCmd.angular.z);
                    NextGoal goal;
                    goal.twist.linear.x = baseCmd.linear.x;
                    goal.twist.linear.y = baseCmd.linear.y;
                    goal.twist.angular.z = baseCmd.angular.z;
                    planPublisher.publish(goal);
                }

                // Process any callbacks
                ros::spinOnce();
                updateRate.sleep();
            }

            if(tp.isGoalReached()){
                ROS_DEBUG("Local planner reached goal");
            }
            if(ros::Time::now() >= nextUpdateTime){
                ROS_DEBUG("Exiting loop because next update time reached");
            }
            if(!ros::ok()){
                ROS_INFO("Exiting due to system failure");
            }

            if(i == getPath.response.poses.end() - 1 && tp.isGoalReached()){
                ROS_DEBUG("Path completed successfully");
                break;
            }
        }

        // Stop movement
        ROS_INFO("Stopping movement as movement plan is complete");
        NextGoal goal;
        goal.twist.linear.x = 0.0;
        goal.twist.linear.y = 0.0;
        goal.twist.angular.z = 0.0;
        planPublisher.publish(goal);
        return true;
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_planner");

	PathPlanner ad;
	ros::spin();
}
