#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <boost/algorithm/string/predicate.hpp>
#include <sensor_msgs/JointState.h>
#include <dogsim/utils.h>
#include <dogsim/GetPath.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>

namespace {
using namespace std;

class TotalForceMeasurer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    double totalForce;
    sensor_msgs::JointStateConstPtr lastJointState;
    
    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;
    message_filters::Subscriber<sensor_msgs::JointState> jointStateSub;

 public:
    TotalForceMeasurer() : 
       privateHandle("~"), 
       totalForce(0.0),
       startMeasuringSub(nh, "start_measuring", 1),
       stopMeasuringSub(nh, "stop_measuring", 1),
       jointStateSub(nh, "joint_states", 1){
        startMeasuringSub.registerCallback(
                boost::bind(&TotalForceMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(
                boost::bind(&TotalForceMeasurer::stopMeasuring, this, _1));
    }
  
 private:

    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        ROS_INFO("Total Force measurement initiated");
        jointStateSub.registerCallback(
                boost::bind(&TotalForceMeasurer::callback, this, _1));
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        // Print out the final measurements
        ROS_INFO("Total force(N): %f", totalForce);
    }

    void callback(sensor_msgs::JointStateConstPtr jointState){
      ROS_DEBUG("Received a message @ %f", jointState->header.stamp.toSec());

      // Ignore the first measurement so we can get a clean baseline.
      if(!lastJointState.get()){
        lastJointState = jointState;
        return;
      }
      
      // Determine the time delta
      double deltaSecs = jointState->header.stamp.toSec() - lastJointState->header.stamp.toSec();

      // Iterate over all the joints.
      double deltaForce = 0.0;
      for(unsigned int i = 0; i < jointState->effort.size(); ++i){
        ROS_DEBUG("Joint %s is exerting %f", jointState->name[i].c_str(), jointState->effort[i]);
        if(signbit(jointState->effort[i]) != signbit(lastJointState->effort[i]) && abs(jointState->effort[i]) > 0 && abs(lastJointState->effort[i]) > 0){
            double ratio = abs(jointState->effort[i]) / abs(lastJointState->effort[i]);
            double l1 = ratio * deltaSecs / (1 + ratio);
            double l2 = deltaSecs - l1;
            deltaForce += utils::square(0.5 * jointState->effort[i] * l1) + utils::square(0.5 * lastJointState->effort[i] * l2);
        }
        else {
            // Apply trapezoidal rule
            deltaForce += utils::square(deltaSecs * (jointState->effort[i] + lastJointState->effort[i]) / 2.0);
        }
      }

      totalForce += deltaForce;

      // Save off the message.
      lastJointState = jointState;

      ROS_DEBUG("Delta seconds(s): %f Delta force(N): %f Total force(N): %f", deltaSecs, deltaForce, totalForce);
   }
};
}

int main(int argc, char **argv){
  ros::init(argc, argv, "total_force_measurer");
  TotalForceMeasurer tfm;
  ros::spin();
  return 0;
}

