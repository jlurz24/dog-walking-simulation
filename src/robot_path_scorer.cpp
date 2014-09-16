#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/utils.h>
#include <dogsim/GetEntireRobotPath.h>
#include <common/common.hh>
#include <physics/physics.hh>
#include <message_filters/subscriber.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>

namespace {
    using namespace std;
    using namespace ros;

class RobotPathScorer {
  private:
    NodeHandle nh;
    NodeHandle privateHandle;
    double totalDistanceDeviation;

    unsigned int n;
    Timer timer;
    Time lastTime;
    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;

 public:
    RobotPathScorer() : 
       privateHandle("~"), 
       totalDistanceDeviation(0),
       n(0),
       startMeasuringSub(nh, "start_measuring", 1),
       stopMeasuringSub(nh, "stop_measuring", 1){
         timer = nh.createTimer(Duration(0.1), &RobotPathScorer::callback, this);
         timer.stop();
         // Wait for the service that will provide us simulated object locations.
         service::waitForService("/gazebo/get_model_state");
         service::waitForService("/dogsim/get_path");

         startMeasuringSub.registerCallback(
                 boost::bind(&RobotPathScorer::startMeasuring, this, _1));
         stopMeasuringSub.registerCallback(
                 boost::bind(&RobotPathScorer::stopMeasuring, this, _1));
    }
    
 private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        timer.start();
        ROS_INFO("Robot Path measurement initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        timer.stop();
        ROS_INFO("Robot Path measurement ended. Total position deviation squared(m): %f", totalDistanceDeviation);
    }

    void callback(const TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());
 
      ServiceClient getPathClient = nh.serviceClient<dogsim::GetEntireRobotPath>("/dogsim/get_entire_robot_path");
      dogsim::GetEntireRobotPath getPath;
      getPath.request.increment = 0.1;
      getPathClient.call(getPath);

      ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gazebo_msgs::GetModelState modelState;
      modelState.request.model_name = "pr2";
      modelStateServ.call(modelState);
     
      // Iterate until we find a point closest to the current time.
      vector<geometry_msgs::PoseStamped>::const_iterator j;
      for (j = getPath.response.poses.begin(); j != getPath.response.poses.end(); ++j) {
          if (j->header.stamp > ros::Time::now()) {
              break;
          }
      }

      // Check the goal for the current time.
      gazebo::math::Vector3 gazeboGoal;
      gazeboGoal.x = j->pose.position.x;
      gazeboGoal.y = j->pose.position.y;
      gazeboGoal.z = j->pose.position.z;

      gazebo::math::Vector3 actual(modelState.response.pose.position.x, modelState.response.pose.position.y, modelState.response.pose.position.z);
      double currPositionDeviation = gazeboGoal.Distance(actual);

      // Increase number of samples
      n++;

      // Update the sum squared.
      double duration = timerEvent.current_real.toSec() - lastTime.toSec();
      totalDistanceDeviation += utils::square(currPositionDeviation) * duration;

      lastTime = timerEvent.current_real;
      ROS_DEBUG("Current Robot Position Deviation(m): %f, Total Position Deviation squared(m): %f, Duration(s): %f", currPositionDeviation, totalDistanceDeviation, duration);
   }
};
}

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_path_scorer");
  RobotPathScorer ps;
  ros::spin();
  return 0;
}

