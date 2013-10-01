#pragma once
#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include <boost/math/constants/constants.hpp>
#include <vector>
#include <tf2/LinearMath/btVector3.h>

namespace {
  using namespace std;
  static const double pi = boost::math::constants::pi<double>();
  
  static const double VELOCITY = 0.25; // m/s
  
  class PiecewiseLinearPathProvider : public PathProvider {
      public:
        PiecewiseLinearPathProvider(){
        }
        
        virtual void init(){
            segments = getSegments();
            calculateTotalLength();
        }
    
        virtual double getMaximumTime() const {
            return totalDuration;
        }
        
        virtual geometry_msgs::PointStamped positionAtTime(const double t) const {
            assert(t >= 0.0);
            
            double distance = VELOCITY * t;
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
                // ROS_INFO("Quadrant %u Clockwise %i", quadrant, clockwise ? 1 : -1);
                // ROS_INFO("Sx %f sy %f s2x %f s2y %f", segment1.x(), segment1.y(), segment2.x(), segment2.y());
                if(!clockwise){
                    a *= -1;
                }
                a += pi / 2.0 * quadrant;
                
                // Now add the circular radius

                btVector3 rounding(center.x() + ROUNDING_DISTANCE * cos(a), center.y() + ROUNDING_DISTANCE * -sin(a), 0);
                result = rounding;
            }
            
            geometry_msgs::PointStamped goal;
            goal.header.frame_id = "/map";
            goal.header.stamp = ros::Time::now();
            goal.point.x = result.x() + 1.0; // Offset the start position
            goal.point.y = result.y();
            goal.point.z = 0;
            return goal;
        }
        
    protected:
    
        virtual vector<btVector3> getSegments() const = 0;
        
        void calculateTotalLength(){
            // Calculate the total length
            for(unsigned int i = 0; i < segments.size(); ++i){
              totalLength += segments[i].w();
            }

            totalDuration = totalLength / VELOCITY;
        }
        
     private:
        double totalDuration;
        double totalLength;
        
        vector<btVector3> segments;
        
        static const double ROUNDING_DISTANCE = 1.0;
  };
}
