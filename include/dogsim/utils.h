#include <boost/math/constants/constants.hpp>
#include <common/common.hh>
#include <physics/physics.hh>
#include <geometry_msgs/Point.h>

namespace utils {

  static double pointToPointDistanceSqr(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2){
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
  }

  static double pointToPointDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2){
    return sqrt(pointToPointDistanceSqr(p1, p2));
  }

  static gazebo::math::Vector3 lissajous(const double t){
      // Lissajous parameters.
      static const double a = sqrt(2);
      static const double delta = boost::math::constants::pi<long double>() / 2.0;
      static const double A = 5.0;
      static const double B = 1.5;
      static const double b = 2 * a;

      gazebo::math::Vector3 goal;
      goal.x = A * sin(a * t + delta);
      goal.y = B * sin(b * t);
      goal.z = 0.0;
      return goal;
    }
}
