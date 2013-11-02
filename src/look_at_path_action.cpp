#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>

// Generated messages
#include <dogsim/LookAtPathAction.h>

namespace {
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class LookAtPathAction {
public:
	LookAtPathAction(const string& name) :
		as(nh, name, boost::bind(&LookAtPathAction::look, this, _1),
				false), actionName(name), pointHeadClient(
						"/head_traj_controller/point_head_action") {

		as.registerPreemptCallback(
				boost::bind(&LookAtPathAction::preemptCB, this));
		pointHeadClient.waitForServer();
		as.start();
	}

private:

	void preemptCB() {
		ROS_INFO("Preempting the look at path action");

		if (!as.isActive()) {
			ROS_DEBUG("Look at path canceled prior to start");
			return;
		}
		pointHeadClient.cancelGoal();
		as.setPreempted();
	}

	bool look(const dogsim::LookAtPathGoalConstPtr& goal) {

		pr2_controllers_msgs::PointHeadGoal phGoal;
		phGoal.target = goal->futurePathPosition;
		// TODO: Determine exact robot height.
		phGoal.target.point.z = 1.0;
		pointHeadClient.sendGoalAndWait(phGoal);
		as.setSucceeded();
		return true;
	}

protected:
	ros::NodeHandle nh;

	// Actionlib classes
	actionlib::SimpleActionServer<dogsim::LookAtPathAction> as;
	string actionName;
	PointHeadClient pointHeadClient;
	tf::TransformListener tf;
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "look_at_path_action");
	LookAtPathAction action(ros::this_node::getName());
	ros::spin();

	return 0;
}
