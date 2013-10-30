#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/utils.h>
#include <dogsim/GetPath.h>
#include <common/common.hh>
#include <physics/physics.hh>

using namespace std;

class PathScorer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    double totalDistanceDeviation;
    ros::Timer timer;
    ros::Time lastTime;
 public:
    PathScorer() : 
       privateHandle("~"), 
       totalDistanceDeviation(0){
         timer = nh.createTimer(ros::Duration(0.1), &PathScorer::callback, this);
         // Wait for the service that will provide us simulated object locations.
         ros::service::waitForService("/gazebo/get_model_state");
         ros::service::waitForService("/dogsim/get_path");

         timer.start();
         ROS_INFO("Path measurement initiated");
    }
  
    ~PathScorer(){
      ROS_INFO("Path measurement ended. Total position deviation squared(m): %f", totalDistanceDeviation);
    }
    
 private:
    void callback(const ros::TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());
 
      ros::ServiceClient getPathClient = nh.serviceClient<dogsim::GetPath>("/dogsim/get_path");
      dogsim::GetPath getPath;
      getPath.request.time = timerEvent.current_real;
      getPathClient.call(getPath);
     
      if(!getPath.response.started){
        return;
      }

      if(getPath.response.ended){
        // Write out the final result
        ROS_INFO("Total Position Deviation squared(m): %f", totalDistanceDeviation);
        timer.stop();
        return;
      }

      ros::ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
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

int main(int argc, char **argv){
  ros::init(argc, argv, "path_scorer");
  PathScorer ps;
  ros::spin();
  return 0;
}

