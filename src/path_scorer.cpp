#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/utils.h>

using namespace std;

inline double square(const double a){
  return a * a;
}

class PathScorer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    double averagePositionDeviation;
    unsigned int iterations;
    ros::Time startTime;
    ros::Timer timer;
 
 public:
    PathScorer() : 
       privateHandle("~"), 
       averagePositionDeviation(0),
       iterations(0),
       startTime(ros::Time::now()){
         timer = nh.createTimer(ros::Duration(0.25), &PathScorer::callback, this);
         // Wait for the service that will provide us simulated object locations.
         ros::service::waitForService("/gazebo/get_model_state");

         timer.start();
         ROS_INFO("Measurement initiated");
    }
  
    ~PathScorer(){
      ROS_INFO("Measurement ended");
    }
    
 private:
    void callback(const ros::TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());
    
      if(timerEvent.current_real.toSec() / utils::TIMESCALE_FACTOR > 15.0){
        ROS_DEBUG("End of scoring time reached");
        return;
      }

      ros::service::waitForService("/gazebo/get_model_state");
      ros::ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
      gazebo_msgs::GetModelState modelState;
      modelState.request.model_name = "dog";
      modelStateServ.call(modelState);
     
      // Check the goal for the current time.
      // TODO: Be smarter about making time scale based on velocity.
      gazebo::math::Vector3 gazeboGoal = utils::lissajous(timerEvent.current_real.toSec() / utils::TIMESCALE_FACTOR);
      gazebo::math::Vector3 actual(modelState.response.pose.position.x, modelState.response.pose.position.y, modelState.response.pose.position.z);
      double currPositionDeviation = gazeboGoal.Distance(actual);

      // Update the moving averages.
      averagePositionDeviation = (currPositionDeviation + iterations * averagePositionDeviation) / (iterations + 1);
      iterations++;
      double duration = std::max(ros::Time::now().toSec() - startTime.toSec(), 0.1);
      ROS_INFO("Current Position Deviation(m): %f, Average Position Deviation(m): %f, Duration(s): %f", currPositionDeviation, averagePositionDeviation, duration);
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "path_scorer");
  PathScorer ps;
  ros::spin();
  return 0;
}

