#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>

namespace {
    
  //! Factor to slow down the lissajous calculation.
  //! TODO: Be smarter about making time scale based on velocity.
  const double TIMESCALE_FACTOR = 75.0;

  //! Amount of time it takes to perform a full lissajous cycle.
  const double FULL_CYCLE_T = 4.45;

  class LissajousPathProvider : public PathProvider {
      public:
        LissajousPathProvider(){
        }
        virtual ~LissajousPathProvider(){}
        
        virtual void init(){
        }
    
        virtual ros::Duration getMaximumTime() const {
            return ros::Duration(FULL_CYCLE_T * TIMESCALE_FACTOR);
        }
        
      protected:
        virtual geometry_msgs::Point positionAtTime(const ros::Duration baseT) const {
            // Lissajous parameters.
            static const double a = sqrt(2);
            static const double delta = boost::math::constants::pi<long double>() / 2.0;
            static const double A = 6.0;
            static const double B = 3.0;
            static const double b = 2 * a;
            double t = baseT.toSec() / TIMESCALE_FACTOR;
            
            geometry_msgs::Point position;
            position.y = (A * sin(a * t + delta)) - 7; // Offset the start and invert;
            position.x = B * sin(b * t) + 1;
            position.z = 0.0;
            return position;
        }
  };
}
