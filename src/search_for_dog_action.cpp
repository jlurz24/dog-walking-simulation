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
						"/head_traj_controller/point_head_action", true) {

		as.registerPreemptCallback(
				boost::bind(&SearchForDogAction::preemptCB, this));
		pointHeadClient.waitForServer();
		as.start();
	}

private:

	void preemptCB() {
		ROS_INFO("Preempting the search for dog action");

		if (!as.isActive()) {
			ROS_DEBUG("Search for dog action canceled prior to start");
			return;
		}
		pointHeadClient.cancelGoal();
		as.setPreempted();
	}

	bool search(const dogsim::SearchForDogGoalConstPtr& goal) {
		ROS_INFO("Requesting search on search for dog action");

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
				handInHandFrame.header.frame_id = "r_wrist_roll_link";
				try {
					tf.transformPoint("/base_footprint", ros::Time(0),
							handInHandFrame, handInHandFrame.header.frame_id,
							handInBaseFrame);
				} catch (tf::TransformException& ex) {
					ROS_ERROR(
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

		pointHeadClient.sendGoal(phGoal);
        if(!pointHeadClient.waitForResult(ros::Duration(1.0))){
            ROS_INFO("Look for dog timed out");
            pointHeadClient.cancelGoal();
            as.setAborted();
            return false;
        }

        if(pointHeadClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Look for dog succeeded");
            as.setSucceeded();
            return true;
        }

        ROS_INFO("Look for dog failed: %s", pointHeadClient.getState().toString().c_str());
        as.setAborted();
        return false;

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
