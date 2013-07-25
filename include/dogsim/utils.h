#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>

namespace utils {
  static inline double square(const double a){
    return a * a;
  }

  static inline visualization_msgs::Marker createMarker(const geometry_msgs::Point& position, const std_msgs::Header& header, std_msgs::ColorRGBA& color, bool persist){
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
     arrow.scale.x = arrow.scale.y = arrow.scale.z = 1.0;
     arrow.color = color;
     return arrow;
 }
}
