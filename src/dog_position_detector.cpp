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

const unsigned int UNKNOWN_ID = std::numeric_limits<unsigned int>::max();
const double STALE_THRESHOLD_DEFAULT = 1.0;
const double LEASH_STRETCH_ERROR_DEFAULT = 0.25;
const double DOG_HEIGHT_ERROR_DEFAULT = 0.25;
const double DOG_HEIGHT_DEFAULT = 0.1;

typedef vector<position_tracker::DetectedDynamicObject> DetectedDynamicObjectsList;

double distance2(const PointStamped& a, const PointStamped &b) {
    return utils::square(a.point.x - b.point.x)
            + utils::square(a.point.y - b.point.y)
            + utils::square(a.point.z - b.point.z);
}

double totalVelocity(const position_tracker::DetectedDynamicObject& obj){
    return fabs(obj.velocity.twist.linear.x) + fabs(obj.velocity.twist.linear.y) + fabs(obj.velocity.twist.linear.z);
}

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
        double d = sqrt(distance2(positionInBaseFrame, handPosition));
        ROS_DEBUG("Distance to possible dog position from hand: %f", d);
        return d > leashThreshold;
    }

};

struct OverMaxDogHeight {

    const double dogHeightThreshold;
    const tf::TransformListener& tf;

    OverMaxDogHeight(const double dogHeight, const double dogHeightError, const tf::TransformListener& tf):
        dogHeightThreshold(dogHeight * (1 + dogHeightError)),
        tf(tf){
    }

    bool operator()(const position_tracker::DetectedDynamicObject& obj) {
        // Convert to base frame.
        PointStamped positionInBaseFrame;
        tf.transformPoint("base_footprint", obj.position, positionInBaseFrame);

        ROS_DEBUG("Height of measurement in base frame: %f", positionInBaseFrame.point.z);
        return positionInBaseFrame.point.z > dogHeightThreshold;
    }

};

struct DistanceFromHand {
    const PointStamped& handPosition;

    DistanceFromHand(const PointStamped& aHandPosition) : handPosition(aHandPosition){
    }

    bool operator()(const position_tracker::DetectedDynamicObject& a, const position_tracker::DetectedDynamicObject& b){
        return distance2(a.position, handPosition) < distance2(b.position, handPosition);
    }
};

struct Velocity {

    Velocity(){
    }

    bool operator()(const position_tracker::DetectedDynamicObject& a, const position_tracker::DetectedDynamicObject& b){
        // Frame is irrelevant
        // Detection of angular velocity is not current implmented.
        return totalVelocity(a) < totalVelocity(b);
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

    //! When to consider an observation stale
    ros::Duration staleThreshold;

    //! Height of the dog
    double dogHeight;

    //! Amount of error in the dog height
    double dogHeightError;
    static const double MIN_DETECTABLE_VELOCITY = 0.01;

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

        // Global parameter
        nh.param("leash_length", leashLength, 2.0);

        // Use a fairly large error here as the arm may be currently moving
        // which will impact this distance
        pnh.param("leash_stretch_error", leashStretchError, LEASH_STRETCH_ERROR_DEFAULT);

        nh.param<double>("dog_height", dogHeight, DOG_HEIGHT_DEFAULT);
        pnh.param("dog_height_error", dogHeightError, DOG_HEIGHT_ERROR_DEFAULT);

        double staleThresholdD;
        pnh.param("stale_threshold", staleThresholdD, STALE_THRESHOLD_DEFAULT);
        staleThreshold.fromSec(staleThresholdD);

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

            // Convert positions to /base_footprint
            tf.waitForTransform("/base_footprint", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
            DetectedDynamicObjectsList possiblePositions = msg->objects;

            for(unsigned int i = 0; i < possiblePositions.size(); ++i){
                tf.transformPoint("/base_footprint", possiblePositions[i].position, possiblePositions[i].position);
            }

            PointStamped handInBaseFrame = findHandInBaseFrame();

            // Apply definitive filters to the possible points. These filters
            // eliminate points that cannot possibly be the correct point.

            possiblePositions.erase(
                    std::remove_if(possiblePositions.begin(), possiblePositions.end(),
                            OutOfLeashDistance(leashLength, leashStretchError, handInBaseFrame, tf)), possiblePositions.end());

            ROS_DEBUG("%lu possible dog positions at end of distance filtering", possiblePositions.size());

            possiblePositions.erase(
                    std::remove_if(possiblePositions.begin(), possiblePositions.end(),
                            OverMaxDogHeight(dogHeight, dogHeightError, tf)), possiblePositions.end());

            ROS_DEBUG("%lu possible dog positions at end of dog height filtering", possiblePositions.size());


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
                    ROS_DEBUG("Possible position filter did not reduce number of possible positions to 1");
                    match = std::find_if(possiblePositions.begin(), possiblePositions.end(), MatchesID(lastId));

                    if(match == possiblePositions.end()){
                        ROS_WARN("No possible position matching the last id found. Applying velocity filter");
                        match = std::max_element(possiblePositions.begin(), possiblePositions.end(), Velocity());

                        if (totalVelocity(*match) < MIN_DETECTABLE_VELOCITY) {
                            ROS_WARN("No moving objects found. Applying closest position filter");
                            // Select the closest
                            match = std::min_element(possiblePositions.begin(), possiblePositions.end(), DistanceFromHand(handInBaseFrame));
                        }
                    }
                }
                lastId = (*match).id;
                dogPositionMsg.pose.header = (*match).position.header;
                assert(dogPositionMsg.pose.header.frame_id.size() > 0 && "frame_id was not set");
                dogPositionMsg.pose.pose.position = (*match).position.point;
                dogPositionMsg.unknown = false;
                dogPositionMsg.measuredTime = (*match).measuredTime;
                dogPositionMsg.stale = ((*match).position.header.stamp - (*match).measuredTime > staleThreshold);
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
