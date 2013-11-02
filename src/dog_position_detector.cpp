#include <ros/ros.h>
#include <dogsim/DogPosition.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <message_filters/subscriber.h>

namespace {
using namespace std;
using namespace dogsim;

class DogPositionDetector {
private:
    //! Publisher for the dog position
    ros::Publisher dogPositionPub;

    //! The node handle we'll be using
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Dog blob messages
    message_filters::Subscriber<position_tracker::DetectedDynamicObjects> objectSub;

public:
    //! ROS node initialization
    DogPositionDetector() :
        pnh("~"), objectSub(nh, "object_tracks/dog/positions_velocities", 1) {

        ros::SubscriberStatusCallback connectCB = boost::bind(&DogPositionDetector::startListening,
                this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &DogPositionDetector::stopListening, this);

        dogPositionPub = nh.advertise<DogPosition>("/dog_position_detector/dog_position", 1,
                connectCB, disconnectCB);
        objectSub.registerCallback(boost::bind(&DogPositionDetector::callback, this, _1));
        objectSub.unsubscribe();
    }

private:

    void stopListening() {
        if (dogPositionPub.getNumSubscribers() == 0) {
            ROS_DEBUG("Stopping listeners for DogPositionDetector");
            objectSub.unsubscribe();
        }
    }

    void startListening() {
        if (dogPositionPub.getNumSubscribers() == 1) {
            ROS_DEBUG("Starting listeners for DogPositionDetector");
            objectSub.subscribe();
        }
    }

    void callback(const position_tracker::DetectedDynamicObjectsConstPtr msg) {

        DogPosition dogPositionMsg;

        if (msg->positions.size() == 0) {
            ROS_DEBUG("Dog position is unknown");
            dogPositionMsg.unknown = true;
        }
        else {
            // TODO: Handle multiple positions in the message and determine which is the dog.
            ROS_DEBUG(
                    "Received dog position message in %s frame", msg->positions[0].header.frame_id.c_str());
            dogPositionMsg.pose.header = msg->positions[0].header;
            ROS_DEBUG( "Dog position message has %lu positions", msg->positions.size());

            dogPositionMsg.pose.pose.position = msg->positions[0].point;
            dogPositionMsg.unknown = false;
        }

        ROS_DEBUG("Publishing a dog position event");
        dogPositionPub.publish(dogPositionMsg);
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dog_position_detector");

    DogPositionDetector positionDetector;
    ros::spin();
    ROS_INFO("Exiting Dog Position Detector");
    return 0;
}
