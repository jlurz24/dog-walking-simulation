#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <dogsim/utils.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
#include <tf/transform_listener.h>
#include <dogsim/LeashInfo.h>

namespace {
using namespace std;

class LeashForceMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    double totalForce;

    double meanLeashStretch;
    double m2LeashStretch;
    double maxLeashStretch;

    double leashLength;

    unsigned int n;

    dogsim::LeashInfoConstPtr lastLeashState;

    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;
    message_filters::Subscriber<dogsim::LeashInfo> leashSub;

public:
    LeashForceMeasurer() :
        privateHandle("~"),
        totalForce(0.0),
        meanLeashStretch(0),
        m2LeashStretch(0),
        maxLeashStretch(0),
        n(0),
        startMeasuringSub(nh, "start_measuring", 1), stopMeasuringSub(
                nh, "stop_measuring", 1), leashSub(nh, "leash_model/info", 1) {
        startMeasuringSub.registerCallback(
                boost::bind(&LeashForceMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(
                boost::bind(&LeashForceMeasurer::stopMeasuring, this, _1));

        nh.param("leash_length", leashLength, 1.5);
    }

private:

    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        ROS_INFO("Leash Force measurement initiated");
        leashSub.registerCallback(boost::bind(&LeashForceMeasurer::callback, this, _1));
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        // Print out the final measurements
        ROS_INFO("Total leash force(N): %f", totalForce);

        double leashLengthVariance = m2LeashStretch / (n - 1);
        ROS_INFO("Mean Leash Stretch: %f, Leash Stretch Variance: %f, Maximum Leash Stretch: %f", meanLeashStretch, leashLengthVariance, maxLeashStretch);

        leashSub.unsubscribe();
    }

    void callback(const dogsim::LeashInfoConstPtr leashState) {
        ROS_DEBUG("Received a message @ %f", leashState->header.stamp.toSec());

        // Ignore the first measurement so we can get a clean baseline.
        if (!lastLeashState.get()) {
            lastLeashState = leashState;
            return;
        }

        // Increase number of samples
        n++;

        // Determine the time delta
        double deltaSecs = leashState->header.stamp.toSec() - lastLeashState->header.stamp.toSec();

        // Convert to tfVector
        tf::Vector3 forceVector;
        tf::vector3MsgToTF(leashState->force, forceVector);

        tf::Vector3 prevForceVector;
        tf::vector3MsgToTF(lastLeashState->force, prevForceVector);

        // Apply trapezoidal rule
        double deltaForce = deltaSecs * (forceVector.length() + prevForceVector.length()) / 2.0;

        totalForce += deltaForce;

        double deltaP = max(leashState->distance - leashLength, 0.0) - meanLeashStretch;
        meanLeashStretch += deltaP / double(n);
        m2LeashStretch += utils::square(deltaP);
        maxLeashStretch = max(deltaP, maxLeashStretch);

        // Save off the message.
        lastLeashState = leashState;

        ROS_DEBUG("Delta seconds(s): %f Delta force(N): %f Total force(N): %f", deltaSecs,
                deltaForce, totalForce);
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "leash_force_measurer");
    LeashForceMeasurer tfm;
    ros::spin();
    return 0;
}

