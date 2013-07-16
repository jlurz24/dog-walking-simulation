#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/utils.h>

using namespace std;

class PathScorer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    double totalDistanceDeviation;
    unsigned int iterations;
    ros::Timer timer;
 
 public:
    PathScorer() : 
       privateHandle("~"), 
       totalDistanceDeviation(0),
       iterations(0){
         timer = nh.createTimer(ros::Duration(0.1), &PathScorer::callback, this);
         // Wait for the service that will provide us simulated object locations.
         ros::service::waitForService("/gazebo/get_model_state");

         timer.start();
         ROS_INFO("Measurement initiated");
    }
  
    ~PathScorer(){
      ROS_INFO("Measurement ended. Total position deviation: %f", totalDistanceDeviation);
    }
    
 private:
    void callback(const ros::TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());

      bool isStarted;
      nh.param<bool>("path/started", isStarted, false);
      bool isEnded;
      nh.param<bool>("path/ended", isEnded, false);
    
      if(!isStarted){
        return;
      }

      if(isEnded){
        // Write out the final result
        ROS_INFO("Total Position Deviation squared(m): %f", totalDistanceDeviation);
      }

      ros::ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gazebo_msgs::GetModelState modelState;
      modelState.request.model_name = "dog";
      modelStateServ.call(modelState);
     
      // Check the goal for the current time.
      double startTime;
      nh.getParam("path/start_time", startTime); 
      gazebo::math::Vector3 gazeboGoal = utils::lissajous((timerEvent.current_real.toSec() - startTime) / utils::TIMESCALE_FACTOR);

      gazebo::math::Vector3 actual(modelState.response.pose.position.x, modelState.response.pose.position.y, modelState.response.pose.position.z);
      double currPositionDeviation = gazeboGoal.Distance(actual);

      // Update the sum squared.
      totalDistanceDeviation += utils::square(currPositionDeviation);
      iterations++;
      double duration = std::max(ros::Time::now().toSec() - startTime, 0.1);
      ROS_INFO("Current Position Deviation(m): %f, Total Position Deviation squared(m): %f, Duration(s): %f", currPositionDeviation, totalDistanceDeviation, duration);
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "path_scorer");
  PathScorer ps;
  ros::spin();
  return 0;
}

