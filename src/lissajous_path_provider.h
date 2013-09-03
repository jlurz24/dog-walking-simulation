#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>

namespace {
  class LissajousPathProvider : public PathProvider {
      public:
        LissajousPathProvider(const double totalTime){
            // TODO: Use this parameter.
        }
    
        virtual geometry_msgs::PointStamped positionAtTime(const double t) const {
            // Lissajous parameters.
            static const double a = sqrt(2);
            static const double delta = boost::math::constants::pi<long double>() / 2.0;
            static const double A = 15.0;
            static const double B = 6.0;
            static const double b = 2 * a;

            geometry_msgs::PointStamped goal;
            goal.header.frame_id = "/map";
            goal.point.x = -(A * sin(a * t + delta)) + 16.5; // Offset the start and invert;
            goal.point.y = B * sin(b * t);
            goal.point.z = 0.0;
            return goal;
        }
  };
}
