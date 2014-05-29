#pragma once
#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>
#include <vector>
#include <tf2/LinearMath/Vector3.h>

namespace {
  using namespace std;

  const double ROUNDING_DISTANCE = 1.0;

  const double pi = boost::math::constants::pi<double>();
  
  const double VELOCITY = 0.125; // m/s
  
  class PiecewiseLinearPathProvider : public PathProvider {
      public:
        PiecewiseLinearPathProvider(){
        }
        
        virtual ~PiecewiseLinearPathProvider(){}

        virtual void init(){
            segments = getSegments();
            calculateTotalLength();
        }
    
        virtual ros::Duration getMaximumTime() const {
            return ros::Duration(totalDuration);
        }
        
        virtual geometry_msgs::PointStamped positionAtTime(const ros::Duration t) const {
            geometry_msgs::PointStamped goal;
            goal.header.frame_id = "/map";
            goal.header.stamp = ros::Time::now();
            
            if(t.toSec() < 0){
                return goal;
            }
            
            double distance = VELOCITY * t.toSec();
            tf2::Vector3 result = tf2::Vector3(0, 0, 0);
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
                    result += segments[i] * tf2Scalar(segments[i].w() - ROUNDING_DISTANCE);
                    roundingRequired = true;
                    lastSegmentNumber = i;
                    firstRoundingSegment = true;
                }
                // Beginning of rounded segment
                else if(distance < ROUNDING_DISTANCE && i != 0){
                    result += segments[i] * tf2Scalar(ROUNDING_DISTANCE);
                    roundingRequired = true;
                    lastSegmentNumber = i;
                }
                else {
                    result += segments[i] * tf2Scalar(distance);
                }
                break;
              }
            }

            if(roundingRequired){
                // Find the center point.
                tf2::Vector3 center;
                if(firstRoundingSegment){
                    center = result + segments[lastSegmentNumber + 1] * tf2Scalar(ROUNDING_DISTANCE);
                }
                else {
                    center = result + segments[lastSegmentNumber - 1] * -1 * tf2Scalar(ROUNDING_DISTANCE);
                }
                double a;
                double ratio;
                tf2::Vector3 segment1, segment2;
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
                bool clockwise;
                if(segment1.y() > 0){
                    quadrant = segment2.x() > 0 ? 2 : 0;
                    clockwise = segment2.x() > 0 ? true : false;
                }
                else if(segment1.y() < 0){
                    quadrant = segment2.x() > 0 ? 3 : 0;
                    clockwise = segment2.x() > 0 ? false : true;
                }
                // Previous segment was an x segment
                else {
                    if(segment1.x() > 0){
                        quadrant = segment2.y() > 0 ? 1: 3;
                        clockwise = segment2.y() > 0 ? false : true;
                    }
                    else {
                        quadrant = segment2.y() > 0 ? 1: 2;
                        clockwise = segment2.y() ? true : false;

                    }
                }

                if(!clockwise){
                    a *= -1;
                }
                a += pi / 2.0 * quadrant;
                
                // Now add the circular radius

                tf2::Vector3 rounding(center.x() + ROUNDING_DISTANCE * cos(a), center.y() + ROUNDING_DISTANCE * -sin(a), 0);
                result = rounding;
            }
            

            goal.point.x = result.x() + 1.0; // Offset the start position
            goal.point.y = result.y();
            goal.point.z = 0;
            return goal;
        }
        
    protected:
    
        virtual vector<tf2::Vector3> getSegments() const = 0;
        
        void calculateTotalLength(){
            // Calculate the total length
            double totalLength = 0;
            for(unsigned int i = 0; i < segments.size(); ++i){
              totalLength += segments[i].w();
            }

            totalDuration = totalLength / VELOCITY;
        }
        
     private:
        double totalDuration;
        
        vector<tf2::Vector3> segments;
  };
}
