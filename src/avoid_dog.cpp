#include <ros/ros.h>
#include <dogsim/DogPosition.h>
#include <dogsim/AvoidingDog.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <dogsim/MoveDogAwayAction.h>
#include <actionlib/client/simple_action_client.h>

namespace {
using namespace std;
using namespace dogsim;

//! Radius of the robot to edge of the square base.
static const double BASE_RADIUS = 0.668 / 2.0;

//! Space to keep in front of the robot in meters
static const double FRONT_AVOIDANCE_THRESHOLD = 0.50;

typedef actionlib::SimpleActionClient<MoveDogAwayAction> MoveDogAwayClient;

class AvoidDog {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

	//! Dog position subscriber
	auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

	//! Client for moving the dog out of the way
	MoveDogAwayClient moveDogAwayClient;

	//! Transform listener
	tf::TransformListener tf;

	//! Publisher for avoiding dog messages
	ros::Publisher avoidingDogPub;

	//! Whether we are currently avoiding the dog
	bool avoidingDog;

public:
	AvoidDog() :
		pnh("~"), moveDogAwayClient("move_dog_away_action", true), avoidingDog(
				false) {

		dogPositionSub.reset(
				new message_filters::Subscriber<DogPosition>(nh,
						"dog_position_detector/dog_position", 1));
		dogPositionSub->registerCallback(
				boost::bind(&AvoidDog::dogPositionCallback, this, _1));

		avoidingDogPub = nh.advertise<AvoidingDog>("/avoid_dog/avoiding", 1);
		moveDogAwayClient.waitForServer();
	}

	void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
		ROS_DEBUG(
				"Received a dog position callback @ %f", ros::Time::now().toSec());
		if (dogPosition->unknown) {
			ROS_DEBUG("Dog position is unknown - cannot avoid");
			return;
		}

		// Convert the positions to the robot frame.
		geometry_msgs::PoseStamped dogPoseInBaseFrame;
		if (dogPosition->pose.header.frame_id != "/base_footprint") {
			try {
				tf.transformPose("/base_footprint", ros::Time(0),
						dogPosition->pose, dogPosition->pose.header.frame_id,
						dogPoseInBaseFrame);
			} catch (...) {
				ROS_ERROR(
						"Failed to transform pose in DogAvoider from %s to /base_footprint", dogPosition->pose.header.frame_id.c_str());
				return;
			}
		} else {
			dogPoseInBaseFrame = dogPosition->pose;
		}
		// Determine if we should avoid the dog.
		bool dogInFront = dogPoseInBaseFrame.pose.position.x
				< (FRONT_AVOIDANCE_THRESHOLD - BASE_RADIUS)
				&& dogPoseInBaseFrame.pose.position.x >= BASE_RADIUS
				&& abs(dogPoseInBaseFrame.pose.position.y) < BASE_RADIUS;
		if (dogInFront) {
			ROS_INFO(
					"Attempting to avoid dog @ %f %f with FAT %f and BR = %f", dogPoseInBaseFrame.pose.position.x, dogPoseInBaseFrame.pose.position.y, FRONT_AVOIDANCE_THRESHOLD, BASE_RADIUS);

			// Notify that we are avoiding the dog.
			AvoidingDog msg;
			msg.pose = dogPoseInBaseFrame;
			msg.avoiding = true;
			avoidingDogPub.publish(msg);

			if (!avoidingDog) {
				ROS_INFO("Activating the move dog away client");
				MoveDogAwayGoal goal;
				moveDogAwayClient.sendGoal(goal);
			}
			avoidingDog = true;
		} else if (!dogInFront) {
			if (avoidingDog) {
				ROS_INFO("Deactivating the move dog away client");
				moveDogAwayClient.cancelGoal();
			}
			avoidingDog = false;

			// Notify that we are done avoiding
			AvoidingDog msg;
			msg.pose = dogPoseInBaseFrame;
			msg.avoiding = false;
			avoidingDogPub.publish(msg);
		}

		ROS_DEBUG(
				"Dog position callback completed @ %f", ros::Time::now().toSec());
	}
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "avoid_dog");

	AvoidDog ad;
	ros::spin();
}
