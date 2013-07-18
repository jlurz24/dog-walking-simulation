#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

using namespace std;

class SetMaxUpdateRate {
public:
  SetMaxUpdateRate(): _pnh("~"){
    ROS_INFO("Setting the max update rate");
    _pnh.param<double>("max_update_rate", _maxUpdateRate, 1000);
  }
  
  void setMaxUpdateRate(){
    ROS_INFO("Setting the max update rate to %f", _maxUpdateRate);
    // Get the current values.
    ros::ServiceClient getClient = _nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    getClient.waitForExistence();
    
    gazebo_msgs::GetPhysicsProperties::Request getReq;
    gazebo_msgs::GetPhysicsProperties::Response getRes;

    if(!getClient.call(getReq, getRes)){
      ROS_WARN("Failed to get the physics properties");
    }
    else {
      ROS_INFO("Got the physics properties");
    }

    // Now update the max time step
    gazebo_msgs::SetPhysicsProperties::Request setReq;
    setReq.time_step = getRes.time_step;
    setReq.gravity = getRes.gravity;
    setReq.ode_config = getRes.ode_config;
    setReq.max_update_rate = _maxUpdateRate;

    ros::ServiceClient setClient = _nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
    setClient.waitForExistence();
    gazebo_msgs::SetPhysicsProperties::Response setRes;
    if(!setClient.call(setReq, setRes)){
      ROS_WARN("Failed to set the physics properties");
    }
    else {
      ROS_INFO("Set the physics properties");
    }

  }
  
  protected:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    double _maxUpdateRate;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "set_max_update_rate");
  SetMaxUpdateRate setter;
  setter.setMaxUpdateRate();
  return 0;
}
