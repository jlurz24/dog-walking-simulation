#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/DogPosition.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
using namespace std;

inline double square(const double a) {
    return a * a;
}

class DogPositionMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;

    double meanPositionDeviation;
    double m2PositionDeviation;

    double meanTimeDuration;
    double m2TimeDuration;
    unsigned int n;

    ros::Time lastTime;
    ros::Time startTime;
    ros::Duration knownTime;
    ros::Duration unknownTime;

    auto_ptr<message_filters::Subscriber<dogsim::DogPosition> > dogSub;
    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;
    std::string modelName;

public:
    DogPositionMeasurer() :
            pnh("~"),
            meanPositionDeviation(0),
            m2PositionDeviation(0),
            meanTimeDuration(0),
            m2TimeDuration(0),
            n(0),
            startMeasuringSub(nh, "start_measuring", 1),
            stopMeasuringSub(nh, "stop_measuring", 1) {

        pnh.param<string>("model_name", modelName, "dog");

        // Setup the subscriber
        dogSub.reset(
                new message_filters::Subscriber<dogsim::DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogSub->unsubscribe();
        dogSub->registerCallback(boost::bind(&DogPositionMeasurer::callback, this, _1));

        startMeasuringSub.registerCallback(
                boost::bind(&DogPositionMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(
                boost::bind(&DogPositionMeasurer::stopMeasuring, this, _1));
    }

private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        startTime = lastTime = msg->header.stamp;
        dogSub->subscribe();
        ROS_INFO("Measurement of dog position initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        dogSub->unsubscribe();

        double totalTime = msg->header.stamp.toSec() - startTime.toSec();
        ROS_INFO(
                "Total Known Time: %f, Total Unknown Time: %f, Total Time: %f, Percent Known: %f, Percent Unknown: %f",
                knownTime.toSec(), unknownTime.toSec(), totalTime,
                knownTime.toSec() / totalTime * 100, unknownTime.toSec() / totalTime * 100);

        double positionDeviationVariance = m2PositionDeviation / (n - 1);
        ROS_INFO("Mean Position Deviation: %f, Position Variance: %f", meanPositionDeviation,
                positionDeviationVariance);

        double timeDurationVariance = m2TimeDuration / (n - 1);
        ROS_INFO("Mean Time Duration: %f, Time Variance: %f", meanTimeDuration,
                timeDurationVariance);
    }

    void callback(const dogsim::DogPositionConstPtr dogPositionMsg) {
        ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

        ros::Duration timePassed = dogPositionMsg->header.stamp - lastTime;
        lastTime = dogPositionMsg->header.stamp;

        // Check if there are any messages.
        if (dogPositionMsg->unknown) {
            unknownTime += timePassed;
            return;
        }
        knownTime += timePassed;

        // Fetch the model for the true position
        ros::service::waitForService("/gazebo/get_model_state");
        ros::ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>(
                "/gazebo/get_model_state");

        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = modelName;
        modelStateServ.call(modelState);

        // Increase number of samples
        n++;

        const geometry_msgs::Point& knownPosition = modelState.response.pose.position;
        const geometry_msgs::Point& estimatedPosition = dogPositionMsg->pose.pose.position;
        // Do not include z position because it is not tracked
        double positionDeviation = sqrt(
                square(knownPosition.x - estimatedPosition.x)
                        + square(knownPosition.y - estimatedPosition.y)) * timePassed.toSec();

        double deltaP = positionDeviation - meanPositionDeviation;
        meanPositionDeviation += deltaP / double(n);
        m2PositionDeviation += square(deltaP);

        double deltaT = (ros::Time::now().toSec() - dogPositionMsg->header.stamp.toSec()) - meanTimeDuration;
        meanTimeDuration += deltaT / double(n);
        m2TimeDuration += square(deltaT);

        ROS_DEBUG("Current Position Deviation: %f", positionDeviation);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "dog_position_measurer");
    DogPositionMeasurer pvm;
    ros::spin();
    return 0;
}
