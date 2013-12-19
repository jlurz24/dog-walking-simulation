#include <ros/ros.h>
#include <dogsim/utils.h>
#include <dogsim/DogPosition.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>

namespace {
using namespace std;
using namespace dogsim;

class DogVisualizer {
private:
	//! Publisher for the dog position visualization.
	ros::Publisher dogVizPubPerm;

	//! Publisher for the ephemeral dogVizPub;
	ros::Publisher dogVizPubEphem;

	//! Publisher for the dog position
	ros::Publisher dogDirectionVizPub;

	//! The node handle we'll be using
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

	//! Dog position subscriber
	auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;
public:
	//! ROS node initialization
	DogVisualizer() :
		pnh("~") {

		ros::SubscriberStatusCallback connectCB = boost::bind(
				&DogVisualizer::startListening, this);
		ros::SubscriberStatusCallback disconnectCB = boost::bind(
				&DogVisualizer::stopListening, this);

		dogVizPubPerm = nh.advertise<visualization_msgs::Marker>(
				"/dog_position_detector/dog_position_viz_perm", 1,
				connectCB, disconnectCB);
		dogVizPubEphem = nh.advertise<visualization_msgs::Marker>(
				"/dog_position_detector/dog_position_viz_ephem", 1,
				connectCB, disconnectCB);
		dogDirectionVizPub = nh.advertise<visualization_msgs::Marker>(
				"/dog_position_detector/dog_direction_viz", 1, connectCB,
				disconnectCB);

		dogPositionSub.reset(
				new message_filters::Subscriber<dogsim::DogPosition>(nh,
						"/dog_position_detector/dog_position", 1));
		dogPositionSub->registerCallback(
				boost::bind(&DogVisualizer::callback, this, _1));
		dogPositionSub->unsubscribe();
	}

private:
	void stopListening() {
		if (dogVizPubPerm.getNumSubscribers() == 0
				&& dogVizPubEphem.getNumSubscribers() == 0
				&& dogDirectionVizPub.getNumSubscribers() == 0) {
			ROS_DEBUG("Stopping listeners for DogVisualizer");
			dogPositionSub->unsubscribe();
		}
	}

	void startListening() {
		if (dogVizPubPerm.getNumSubscribers()
				+ dogVizPubEphem.getNumSubscribers()
				+ dogDirectionVizPub.getNumSubscribers() == 1) {
			ROS_DEBUG("Starting listeners for DogVizualizer");
			dogPositionSub->subscribe();
		}
	}

	void callback(const dogsim::DogPositionConstPtr msg) {
	    if(msg->unknown){
	        return;
	    }

		const geometry_msgs::PoseStamped& dogPose = msg->pose;

		// Visualize the dog.
		static const std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
		if (dogVizPubPerm.getNumSubscribers() > 0) {
            visualization_msgs::Marker marker = utils::createMarker(dogPose.pose.position,
                    dogPose.header, BLUE, true);
            marker.scale.x = 0.1;
			dogVizPubPerm.publish(
					marker);
		}

		if (dogVizPubEphem.getNumSubscribers() > 0) {
			visualization_msgs::Marker marker;
			marker.header = dogPose.header;
			marker.ns = "dogsim";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose = dogPose.pose;
			marker.color = BLUE;
			marker.scale.x = 0.25;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;

			dogVizPubEphem.publish(marker);
		}

		if (dogDirectionVizPub.getNumSubscribers() > 0) {
			geometry_msgs::QuaternionStamped orientation;
			orientation.quaternion = dogPose.pose.orientation;
			visualization_msgs::Marker marker;
			marker.header = orientation.header;
			marker.ns = "dogsim";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position = dogPose.pose.position;
			marker.pose.orientation = orientation.quaternion;
			marker.color = BLUE;
			marker.color.a = 1;
			marker.scale.x = 0.4;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;
			dogDirectionVizPub.publish(marker);
		}

	}
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "dog_visualizer");

	DogVisualizer viz;
	ros::spin();
	return 0;
}
