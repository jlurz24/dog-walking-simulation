#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <boost/algorithm/string/predicate.hpp>
#include <sensor_msgs/JointState.h>
#include <dogsim/utils.h>
#include <dogsim/GetPath.h>

using namespace std;

class TotalForceMeasurer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    double totalForce;
    sensor_msgs::JointStateConstPtr lastJointState;
    ros::Timer timer;
    
 public:
    TotalForceMeasurer() : 
       privateHandle("~"), 
       totalForce(0.0){
      ROS_INFO("Total Force measurement initiated");
	  ros::service::waitForService("/dogsim/get_path");
      timer = nh.createTimer(ros::Duration(0.1), &TotalForceMeasurer::callback, this);
    }
  
    ~TotalForceMeasurer(){
      ROS_INFO("Total force measurement ended. Total force: %f", totalForce);
    }
    
 private:
    void callback(const ros::TimerEvent& timerEvent){
      ROS_DEBUG("Received a message @ %f", timerEvent.current_real.toSec());

      // Get the current joint state
      sensor_msgs::JointStateConstPtr jointState = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", nh, ros::Duration(0.5));
      
      // Ignore the first measurement so we can get a clean baseline.
      if(!lastJointState.get()){
        lastJointState = jointState;
        return;
      }

      ros::ServiceClient getPathClient = nh.serviceClient<dogsim::GetPath>("/dogsim/get_path");
      dogsim::GetPath getPath;
      getPath.request.time = timerEvent.current_real.toSec();
      getPath.request.start = false;
      getPathClient.call(getPath);

      if(!getPath.response.started){
        return;
      }
      if(getPath.response.ended){
        // Print out the final measurements
        ROS_INFO("Total force(N): %f", totalForce);
        return;
      }

      // Determine the time delta
      double deltaSecs = jointState->header.stamp.toSec() - lastJointState->header.stamp.toSec();

      // Iterate over all the joints.
      double deltaForce = 0.0;
      for(unsigned int i = 0; i < jointState->effort.size(); ++i){
        ROS_DEBUG("Joint %s is exerting %f", jointState->name[i].c_str(), jointState->effort[i]);
        // Apply trapezoidal rule
	    deltaForce += utils::square(deltaSecs * (jointState->effort[i] + lastJointState->effort[i]) / 2.0);
      }

      totalForce += deltaForce;

      // Save off the message.
      lastJointState = jointState;

      ROS_INFO("Delta seconds(s): %f Delta force(N): %f Total force(N): %f Force/Second(N/s):%f", deltaSecs, deltaForce, totalForce, totalForce / (jointState->header.stamp.toSec() - getPath.response.elapsedTime));
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "total_force_measurer");
  TotalForceMeasurer tfm;
  ros::spin();
  return 0;
}

