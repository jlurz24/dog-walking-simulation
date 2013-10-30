#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>

// Generated messages
#include <dogsim/SearchForDogAction.h>

namespace {
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class SearchForDogAction {
public:
	SearchForDogAction(const string& name) :
		as(nh, name, boost::bind(&SearchForDogAction::search, this, _1),
				false), actionName(name), pointHeadClient(
						"/head_traj_controller/point_head_action") {

		as.registerPreemptCallback(
				boost::bind(&SearchForDogAction::preemptCB, this));
		as.start();
	}

private:

	void preemptCB() {
		ROS_DEBUG("Preempting the search ford dog action");

		if (!as.isActive()) {
			ROS_DEBUG("Search for dog action canceled prior to start");
			return;
		}
		pointHeadClient.cancelGoal();
		as.setPreempted();
	}

	bool search(const dogsim::SearchForDogGoalConstPtr& goal) {
		ROS_DEBUG("Searching for dog");

		pr2_controllers_msgs::PointHeadGoal phGoal;
		// If the last position is known, use that as our start point.
		if (goal->anyLastPosition) {
			phGoal.target = goal->lastKnownPosition;
		} else {
			// Use the current right hand position at height 0 as the starting point.
			// Determine the position of the hand in the base frame.
			PointStamped handInBaseFrame;
			{
				PointStamped handInHandFrame;
				handInHandFrame.header.frame_id = "r_wrist_roll_joint";
				try {
					tf.transformPoint("/base_footprint", ros::Time(0),
							handInHandFrame, handInHandFrame.header.frame_id,
							handInBaseFrame);
				} catch (tf::TransformException& ex) {
					ROS_INFO(
							"Failed to transform hand position to /base_footprint");
					as.setAborted();
					return false;
				}
			}
			handInBaseFrame.header.frame_id = "/base_footprint";
			handInBaseFrame.header.stamp = ros::Time::now();
			handInBaseFrame.point.z = 0;
			phGoal.target = handInBaseFrame;
		}
		ROS_INFO("Sending goal to PHC");
		utils::sendGoal(&pointHeadClient, phGoal, nh);
		as.setSucceeded();
		return true;
	}

protected:
	ros::NodeHandle nh;

	// Actionlib classes
	actionlib::SimpleActionServer<dogsim::SearchForDogAction> as;
	string actionName;
	PointHeadClient pointHeadClient;
	tf::TransformListener tf;
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "search_for_dog_action");
	SearchForDogAction action(ros::this_node::getName());
	ros::spin();

	return 0;
}
