#include "path_provider.h"
#include "piecewise_linear_path_provider.h"
#include <vector>
#include <tf2/LinearMath/Vector3.h>

namespace {
  
  class BlockWalkPathProvider : public PiecewiseLinearPathProvider {
      public:
        BlockWalkPathProvider(){}
        virtual ~BlockWalkPathProvider(){}
      protected:
        virtual std::vector<tf2::Vector3> getSegments() const {
            std::vector<tf2::Vector3> segments(8);

            segments[0] = tf2::Vector3(1, 0, 0);
            segments[0].setW(tf2Scalar(4));

            segments[1] = tf2::Vector3(0, 1, 0);
            segments[1].setW(tf2Scalar(4));

            segments[2] = tf2::Vector3(-1, 0, 0);
            segments[2].setW(tf2Scalar(4));

            segments[3] = tf2::Vector3(0, 1, 0);
            segments[3].setW(tf2Scalar(4));
            
            segments[4] = tf2::Vector3(1, 0, 0);
            segments[4].setW(tf2Scalar(4.2));
            
            segments[5] = tf2::Vector3(0, -1, 0);
            segments[5].setW(tf2Scalar(12));
            
            segments[6] = tf2::Vector3(-1, 0, 0);
            segments[6].setW(tf2Scalar(4.2));
            
            segments[7] = tf2::Vector3(0, 1, 0);
            segments[7].setW(tf2Scalar(4));
            return segments;
        }
  };
}
