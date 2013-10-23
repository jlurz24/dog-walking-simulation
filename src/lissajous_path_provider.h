#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>

namespace {
    
  //! Factor to slow down the lissajous calculation.
  //! TODO: Be smarter about making time scale based on velocity.
  static const double TIMESCALE_FACTOR = 90.0;

  //! Amount of time it takes to perform a full lissajous cycle.
  static const double FULL_CYCLE_T = 4.45;
  
  class LissajousPathProvider : public PathProvider {
      public:
        LissajousPathProvider(){
        }
        virtual ~LissajousPathProvider(){}
        
        virtual void init(){
        }
    
        virtual double getMaximumTime() const {
            return FULL_CYCLE_T * TIMESCALE_FACTOR;
        }
        
        virtual geometry_msgs::PointStamped positionAtTime(const double baseT) const {
            // Lissajous parameters.
            static const double a = sqrt(2);
            static const double delta = boost::math::constants::pi<long double>() / 2.0;
            static const double A = 8.0;
            static const double B = 4.0;
            static const double b = 2 * a;
            double t = baseT / TIMESCALE_FACTOR;
            
            geometry_msgs::PointStamped goal;
            goal.header.frame_id = "/map";
            goal.point.x = -(A * sin(a * t + delta)) + 9; // Offset the start and invert;
            goal.point.y = B * sin(b * t);
            goal.point.z = 0.0;
            return goal;
        }
  };
}
