#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
#include <dogsim/PathViewInfo.h>

namespace {
using namespace message_filters;
using namespace std;
using namespace dogsim;

class PathVisibilityMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Time lastTime;
    ros::Time startTime;

    double totalScore;

    auto_ptr<Subscriber<PathViewInfo> > pathViewMetricsSub;

    Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;

public:
    PathVisibilityMeasurer() :
            pnh("~"), totalScore(0), startMeasuringSub(nh, "start_measuring", 1), stopMeasuringSub(nh,
                    "stop_measuring", 1) {

        pathViewMetricsSub.reset(new Subscriber<PathViewInfo>(nh, "/path_visibility_detector/view", 1));

        startMeasuringSub.registerCallback(
                boost::bind(&PathVisibilityMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(
                boost::bind(&PathVisibilityMeasurer::stopMeasuring, this, _1));
    }

private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        startTime = lastTime = msg->header.stamp;
        pathViewMetricsSub->registerCallback(boost::bind(&PathVisibilityMeasurer::callback, this, _1));
        ROS_DEBUG("Measurement of path visibility initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        ROS_DEBUG("Measurement of path visibility completed");
        pathViewMetricsSub.release();
        ROS_INFO("Total path visibility score was %f over %f seconds", totalScore, msg->header.stamp.toSec() - startTime.toSec());
    }

    void callback(const PathViewInfoConstPtr& pathViewMetrics) {
        ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

        ros::Time messageTime = pathViewMetrics->header.stamp;
        ros::Duration timePassed = messageTime - lastTime;

        // Update the score
        double increment = timePassed.toSec() * pathViewMetrics->visibilityRatio;
        ROS_DEBUG("Incrementing total score by %f", increment);
        totalScore += increment;
        lastTime = messageTime;
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visibility_measurer");
    PathVisibilityMeasurer pvm;
    ros::spin();
    return 0;
}
