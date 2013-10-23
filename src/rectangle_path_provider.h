#include "path_provider.h"
#include "piecewise_linear_path_provider.h"
#include <vector>
#include <tf2/LinearMath/btVector3.h>

namespace {
  using namespace std;

  class RectanglePathProvider : public PiecewiseLinearPathProvider {
      public:
        virtual RectanglePathProvider(){}
        
    protected:
        virtual vector<btVector3> getSegments() const {
            vector<btVector3> segments(4);
            segments[0] = btVector3(0, 1, 0);
            segments[0].setW(btScalar(5));

            segments[1] = btVector3(1, 0, 0);
            segments[1].setW(btScalar(10));

            segments[2] = btVector3(0, -1, 0);
            segments[2].setW(btScalar(5));
          
            segments[3] = btVector3(-1, 0, 0);
            segments[3].setW(btScalar(10));
            return segments;
        }
  };
}
