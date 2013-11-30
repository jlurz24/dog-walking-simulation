#include <ros/ros.h>
#include <dogsim/DogPosition.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <position_tracker/DetectedDynamicObject.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <dogsim/utils.h>

namespace {

using namespace std;
using namespace dogsim;
using namespace geometry_msgs;

static const unsigned int UNKNOWN_ID = std::numeric_limits<unsigned int>::max();

typedef vector<position_tracker::DetectedDynamicObject> DetectedDynamicObjectsList;

struct OutOfLeashDistance {

    const double leashThreshold;
    const PointStamped& handPosition;
    const tf::TransformListener& tf;

    OutOfLeashDistance(const double leashLength, const double leashStretchError, const PointStamped& handPosition, const tf::TransformListener& tf):
        leashThreshold(leashLength * (1 + leashStretchError)),
        handPosition(handPosition),
        tf(tf){
    }

    bool operator()(const position_tracker::DetectedDynamicObject& obj) {
        // Convert to base frame.
        PointStamped positionInBaseFrame;
        tf.transformPoint("base_footprint", obj.position, positionInBaseFrame);
        double d = sqrt(utils::square(positionInBaseFrame.point.x - handPosition.point.x) +
                        utils::square(positionInBaseFrame.point.y - handPosition.point.y) +
                        utils::square(positionInBaseFrame.point.z - handPosition.point.z));
        ROS_DEBUG("Distance to possible dog position from hand: %f", d);
        return d > leashThreshold;
    }
};

struct MatchesID {
    unsigned int id;
    MatchesID(unsigned int id): id(id){}
    bool operator()(const position_tracker::DetectedDynamicObject& obj){
        return obj.id == id;
    }
};

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

    //! Transform listener
    tf::TransformListener tf;

    //! Length of the leash
    double leashLength;

    //! Amount of possible stretch in the leash as a ratio of the leash length.
    double leashStretchError;

    //! Last id of the dog
    unsigned int lastId;

public:
    //! ROS node initialization
    DogPositionDetector() :
        pnh("~"),
        objectSub(nh, "object_tracks/dog/positions_velocities", 1),
        lastId(UNKNOWN_ID){

        ros::SubscriberStatusCallback connectCB = boost::bind(&DogPositionDetector::startListening,
                this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &DogPositionDetector::stopListening, this);

        nh.param("leash_length", leashLength, 2.0);
        // Use a fairly large error here as the arm may be currently moving
        // which will impact this distance
        nh.param("leash_stretch_error", leashStretchError, 0.5);

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

    PointStamped findHandInBaseFrame() const {
        // Use the current right hand position at height 0 as the starting point.
        // Determine the position of the hand in the base frame.
        PointStamped handInBaseFrame;
        PointStamped handInHandFrame;
        handInHandFrame.header.frame_id = "r_wrist_roll_link";
        try {
            tf.transformPoint("/base_footprint", ros::Time(0), handInHandFrame,
                    handInHandFrame.header.frame_id, handInBaseFrame);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR(
                    "Failed to transform hand position to /base_footprint");
            throw ex;
        }
        handInBaseFrame.header.frame_id = "/base_footprint";
        handInBaseFrame.header.stamp = ros::Time::now();
        return handInBaseFrame;
    }

    void callback(const position_tracker::DetectedDynamicObjectsConstPtr msg) {

        DogPosition dogPositionMsg;
        dogPositionMsg.header = msg->header;
        if (msg->objects.size() == 0) {
            ROS_DEBUG("No detected dynamic objects. Dog position is unknown.");
            dogPositionMsg.unknown = true;
        }
        else {
            ROS_DEBUG("%lu possible dog positions at beginning of filtering", msg->objects.size());
            PointStamped handInBaseFrame = findHandInBaseFrame();

            // Apply definitive filters to the possible points. These filters
            // eliminate points that cannot possibly be the correct point.
            DetectedDynamicObjectsList possiblePositions = msg->objects;
            possiblePositions.erase(
                    std::remove_if(possiblePositions.begin(), possiblePositions.end(),
                            OutOfLeashDistance(leashLength, leashStretchError, handInBaseFrame, tf)), possiblePositions.end());

            // TODO: Add additional filter operations.
            ROS_DEBUG("%lu possible dog positions at end of filtering", possiblePositions.size());

            // Apply preference filters. These filters distinguish between feasible
            // points.
            if(possiblePositions.size() == 0){
                ROS_INFO("No feasible dog positions after filtering");
                dogPositionMsg.unknown = true;
            }
            else {
                DetectedDynamicObjectsList::const_iterator match;
                if(possiblePositions.size() == 1){
                    ROS_DEBUG("Using single possible position");
                    match = possiblePositions.begin();
                }
                else {
                    ROS_INFO("Filters did not reduce number of possible positions to 1");
                    match = std::find_if(possiblePositions.begin(), possiblePositions.end(), MatchesID(lastId));

                    if(match == possiblePositions.end()){
                        ROS_INFO("No possible position matching the last id found.");
                        // TODO: Select the most recent observation?
                        match = possiblePositions.begin();
                    }
                }
                lastId = (*match).id;
                dogPositionMsg.pose.header = (*match).position.header;
                dogPositionMsg.pose.pose.position = (*match).position.point;
                dogPositionMsg.unknown = false;
                dogPositionMsg.measuredTime = (*match).measuredTime;
            }
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
