#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace utils {
  static inline double square(const double a){
    return a * a;
  }

  inline double pointToPointXYDistanceSqr(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2){
    return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  }

  inline double pointToPointXYDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2){
    return (sqrt(pointToPointXYDistanceSqr(p1, p2)));
  }


  static inline visualization_msgs::Marker createMarker(const geometry_msgs::Point& position, const std_msgs::Header& header, const std_msgs::ColorRGBA& color, bool persist){
      static unsigned int uniqueId = 0;
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "dogsim";
      marker.id = persist ? uniqueId++ : 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = position;
      marker.color = color;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
      return marker;
  }

  static inline std_msgs::ColorRGBA createColor(float r, float g, float b){
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    return color;
  }
  
  static inline std_msgs::ColorRGBA createColor(float r, float g, float b, float a){
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  static inline visualization_msgs::Marker createArrow(const double yaw, const std_msgs::Header& header, const std_msgs::ColorRGBA& color){
     // Publish a visualization arrow.
     visualization_msgs::Marker arrow;
     arrow.header = header;
     arrow.ns = "dogsim";
     arrow.id = 0;
     arrow.type = visualization_msgs::Marker::ARROW;
     arrow.action = visualization_msgs::Marker::ADD;
     arrow.pose.position.x = arrow.pose.position.y = arrow.pose.position.z = 0;
     arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
     arrow.scale.x = 1.0;
     arrow.scale.y = arrow.scale.z = 0.01;
     arrow.color = color;
     return arrow;
 }

    /*
     * initialize a client of the specified type and service name.
     * @param serviceName Name of the service.
     * @return Pointer to an initialized client.
     */
    template<class T>
    static std::auto_ptr<T> initClient(const std::string& serviceName){
      ROS_INFO("Initilizing client for %s", serviceName.c_str());
      std::auto_ptr<T> client(new T(serviceName, true));

      // Wait for the action server to come up
      while(!client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for service %s to come up", serviceName.c_str());
      }
      ROS_INFO("Service %s connected", serviceName.c_str());
      return client;
    }

  /*
   * Send a goal to an action client, wait for a result, and 
   * report success or failure.
   * @param client Action client to send the goal to
   * @param goal Goal to send
   * @return Whether the goal was executed successfully.
   */
  template<class T, class U>
  static bool sendGoal(const T& client, const U& goal, ros::NodeHandle& nh, double timeout = 20.0){
    bool success = false;
    if (nh.ok()){
      client->sendGoal(goal);
      if(!client->waitForResult(ros::Duration(timeout))){
        client->cancelGoal();
        ROS_INFO("Timed out achieving goal");
      }
      else {
        actionlib::SimpleClientGoalState state = client->getState();
        success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success){
          ROS_DEBUG("Action finished: %s",state.toString().c_str());
        }
        else {
          ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }

  }
  else {
    ROS_INFO("Nodehandle is invalid. Not sending action");
  }

  return success;
}

}
