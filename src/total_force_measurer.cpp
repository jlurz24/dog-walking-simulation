#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <boost/algorithm/string/predicate.hpp>
#include <sensor_msgs/JointState.h>

using namespace std;

class TotalForceMeasurer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    double totalForce;
    message_filters::Subscriber<sensor_msgs::JointState> jointsSub;
    sensor_msgs::JointStateConstPtr lastJointState;
    ros::Time startTime;
 public:
    TotalForceMeasurer() : 
       privateHandle("~"), 
       totalForce(0.0),
       jointsSub(nh, "joint_states", 1){
      jointsSub.registerCallback(boost::bind(&TotalForceMeasurer::callback, this, _1));
      ROS_INFO("Total Force measurement initiated");
    }
  
    ~TotalForceMeasurer(){
      ROS_INFO("Total force measurement ended");
    }
    
 private:
    void callback(const sensor_msgs::JointStateConstPtr jointState){
      ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

      // Ignore the first measurement so we can get a clean baseline.
      if(!lastJointState.get()){
        lastJointState = jointState;
        startTime = jointState->header.stamp;
        return;
      }
      // Determine the time delta
      double deltaSecs = jointState->header.stamp.toSec() - lastJointState->header.stamp.toSec();

      // Iterate over all the joints.
      double deltaForce = 0.0;
      // TODO: Determine if all force units are compatible.
      for(unsigned int i = 0; i < jointState->effort.size(); ++i){
        ROS_DEBUG("Joint %s is exerting %f", jointState->name[i].c_str(), jointState->effort[i]);
	deltaForce += deltaSecs * (fabs(jointState->effort[i]) + fabs(lastJointState->effort[i])) / 2.0;
      }

      totalForce += deltaForce;

      // Save off the message.
      lastJointState = jointState;

      ROS_INFO("Delta seconds(s): %f Delta force(N): %f Total force(N): %f Force/Second(N/s):%f", deltaSecs, deltaForce, totalForce, totalForce / (jointState->header.stamp.toSec() - startTime.toSec()));
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "total_force_measurer");
  TotalForceMeasurer tfm;
  ros::spin();
  return 0;
}

