#include <boost/math/constants/constants.hpp>
#include <common/common.hh>
#include <physics/physics.hh>
#include <geometry_msgs/Point.h>

namespace utils {
  // Factor to slow down the lissajous calculation.
  static const double TIMESCALE_FACTOR = 10.0;

  static gazebo::math::Vector3 lissajous(const double t){
      
      // Lissajous parameters.
      static const double a = sqrt(2);
      static const double delta = boost::math::constants::pi<long double>() / 2.0;
      static const double A = 5.0;
      static const double B = 1.5;
      static const double b = 2 * a;

      gazebo::math::Vector3 goal;
      goal.x = -(A * sin(a * t + delta)) + 6.5; // Offset the start and invert;
      goal.y = B * sin(b * t);
      goal.z = 0.0;
      return goal;
    }
}
