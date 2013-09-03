#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>
#include <vector>
#include <tf2/LinearMath/btVector3.h>

namespace {
  using namespace std;
  static const double pi = boost::math::constants::pi<double>();
  
  class RectanglePathProvider : public PathProvider {
      public:
        RectanglePathProvider(const double totalDuration){
            segments.resize(4);
            segments[0] = btVector3(0, 1, 0);
            segments[0].setW(btScalar(5));

            segments[1] = btVector3(1, 0, 0);
            segments[1].setW(btScalar(20));

            segments[2] = btVector3(0, -1, 0);
            segments[2].setW(btScalar(5));
          
            segments[3] = btVector3(-1, 0, 0);
            segments[3].setW(btScalar(20));

            // Calculate the total length
            double totalLength = 0;
            for(unsigned int i = 0; i < segments.size(); ++i){
              totalLength += segments[i].w();
            }

            // Calculate the needed velocity to complete the cycle in the given time.
            velocity = totalLength / totalDuration;
        }
    
        virtual geometry_msgs::PointStamped positionAtTime(const double t) const {

            double distance = velocity * t;
            btVector3 result = btVector3(0, 0, 0);
            bool roundingRequired = false;
            bool firstRoundingSegment = false;
            unsigned int lastSegmentNumber = 0;
            for(unsigned int i = 0; i < segments.size(); ++i){
              // If the point is past the segment, add the whole segment.
              if(distance > segments[i].w()){
                result += segments[i] * segments[i].w();
                distance -= segments[i].w();
              }
              // Otherwise add only the correct piece.
              else {
                // Determine if rounding will be required.
                if(segments[i].w() - distance < ROUNDING_DISTANCE && i != segments.size() - 1){
                    // Only add the distance up to the point where rounding will begin
                    result += segments[i] * btScalar(segments[i].w() - ROUNDING_DISTANCE);
                    roundingRequired = true;
                    lastSegmentNumber = i;
                    firstRoundingSegment = true;
                }
                // Beginning of rounded segment
                else if(distance < ROUNDING_DISTANCE && i != 0){
                    result += segments[i] * btScalar(ROUNDING_DISTANCE);
                    roundingRequired = true;
                    lastSegmentNumber = i;
                }
                else {
                    result += segments[i] * btScalar(distance);
                }
                break;
              }
            }

            if(roundingRequired){
                // Find the center point.
                btVector3 center;
                if(firstRoundingSegment){
                    center = result + segments[lastSegmentNumber + 1] * btScalar(ROUNDING_DISTANCE);
                }
                else {
                    center = result + segments[lastSegmentNumber - 1] * -1 * btScalar(ROUNDING_DISTANCE);
                }
                double a;
                double ratio;
                btVector3 segment1, segment2;
                if(firstRoundingSegment){
                    ratio = 1 - (segments[lastSegmentNumber].w() - distance) / ROUNDING_DISTANCE;
                    a = ratio * pi / 4.0;
                    segment1 = segments[lastSegmentNumber];
                    segment2 = segments[lastSegmentNumber + 1];
                }
                else {
                    ratio = distance / ROUNDING_DISTANCE;
                    a = ratio * pi / 4.0 + pi / 4.0;
                    segment1 = segments[lastSegmentNumber - 1];
                    segment2 = segments[lastSegmentNumber];
                }
                
                // Now shift A to the appropriate quadrant.
                unsigned int quadrant;
                if(segment1.y() > 0){
                    quadrant = segment2.x() > 0 ? 2 : 1;
                }
                else if(segment1.y() < 0){
                    quadrant = segment2.x() > 0 ? 3 : 0;
                }
                // Previous segment was an x segment
                else {
                    if(segment1.x() > 0){
                        quadrant = segment2.y() > 0 ? 0: 3;
                    }
                    else {
                        quadrant = segment2.y() > 0 ? 2: 1;
                    }
                }
                a += pi / 2.0 * quadrant;
                
                // Now add the circular radius
                btVector3 rounding(center.x() + ROUNDING_DISTANCE * cos(a), center.y() + ROUNDING_DISTANCE * -sin(a), 0);
                result = rounding;
            }
            
            geometry_msgs::PointStamped goal;
            goal.header.frame_id = "/map";
            goal.point.x = result.x() + 1.0; // Offset the start position
            goal.point.y = result.y();
            goal.point.z = 0;
            return goal;
        }

     private:
        double velocity;
    
        vector<btVector3> segments;
        
        static  const double ROUNDING_DISTANCE = 1.0;
  };
}
