#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/utils.h>
#include <dogsim/GetPath.h>
#include <common/common.hh>
#include <physics/physics.hh>
#include <message_filters/subscriber.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>

namespace {
    using namespace std;
    using namespace ros;

class PathScorer {
  private:
    NodeHandle nh;
    NodeHandle privateHandle;
    double totalDistanceDeviation;
    Timer timer;
    Time lastTime;
    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;
 public:
    PathScorer() : 
       privateHandle("~"), 
       totalDistanceDeviation(0),
       startMeasuringSub(nh, "start_measuring", 1),
       stopMeasuringSub(nh, "stop_measuring", 1){
         timer = nh.createTimer(Duration(0.1), &PathScorer::callback, this);
         timer.stop();
         // Wait for the service that will provide us simulated object locations.
         service::waitForService("/gazebo/get_model_state");
         service::waitForService("/dogsim/get_path");

         startMeasuringSub.registerCallback(
                 boost::bind(&PathScorer::startMeasuring, this, _1));
         stopMeasuringSub.registerCallback(
                 boost::bind(&PathScorer::stopMeasuring, this, _1));
    }
    
 private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        timer.start();
        ROS_INFO("Path measurement initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        timer.stop();
        ROS_INFO("Path measurement ended. Total position deviation squared(m): %f", totalDistanceDeviation);
    }

    void callback(const TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());
 
      ServiceClient getPathClient = nh.serviceClient<dogsim::GetPath>("/dogsim/get_path");
      dogsim::GetPath getPath;
      getPath.request.time = timerEvent.current_real;
      getPathClient.call(getPath);
     
      assert(getPath.response.started);
      assert(!getPath.response.ended);

      ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gazebo_msgs::GetModelState modelState;
      modelState.request.model_name = "dog";
      modelStateServ.call(modelState);
     
      // Check the goal for the current time.
      gazebo::math::Vector3 gazeboGoal;
      gazeboGoal.x = getPath.response.point.point.x;
      gazeboGoal.y = getPath.response.point.point.y;
      gazeboGoal.z = getPath.response.point.point.z;

      gazebo::math::Vector3 actual(modelState.response.pose.position.x, modelState.response.pose.position.y, modelState.response.pose.position.z);
      double currPositionDeviation = gazeboGoal.Distance(actual);

      // Update the sum squared.
      double duration = timerEvent.current_real.toSec() - lastTime.toSec();
      totalDistanceDeviation += utils::square(currPositionDeviation) * duration;
	  lastTime = timerEvent.current_real;
      ROS_DEBUG("Current Position Deviation(m): %f, Total Position Deviation squared(m): %f, Duration(s): %f", currPositionDeviation, totalDistanceDeviation, duration);
   }
};
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_scorer");
  PathScorer ps;
  ros::spin();
  return 0;
}

