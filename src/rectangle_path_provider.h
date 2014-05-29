#include "path_provider.h"
#include "piecewise_linear_path_provider.h"
#include <vector>
#include <tf2/LinearMath/Vector3.h>

namespace {
  using namespace std;

  class RectanglePathProvider : public PiecewiseLinearPathProvider {
      public:
        RectanglePathProvider(){}
        virtual ~RectanglePathProvider(){}
    protected:
        virtual vector<tf2::Vector3> getSegments() const {
            vector<tf2::Vector3> segments(4);
            segments[0] = tf2::Vector3(1, 0, 0);
            segments[0].setW(tf2Scalar(8));

            segments[1] = tf2::Vector3(0, -1, 0);
            segments[1].setW(tf2Scalar(4));
          
            segments[2] = tf2::Vector3(-1, 0, 0);
            segments[2].setW(tf2Scalar(8));

            segments[3] = tf2::Vector3(0, 1, 0);
            segments[3].setW(tf2Scalar(4));

            return segments;
        }
  };
}
