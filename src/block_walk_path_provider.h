#include "path_provider.h"
#include "piecewise_linear_path_provider.h"
#include <vector>
#include <tf2/LinearMath/btVector3.h>

namespace {
  
  class BlockWalkPathProvider : public PiecewiseLinearPathProvider {
      public:
        BlockWalkPathProvider(){}
        virtual ~BlockWalkPathProvider(){}
      protected:
        virtual std::vector<btVector3> getSegments() const {
            std::vector<btVector3> segments(8);

            segments[0] = btVector3(1, 0, 0);
            segments[0].setW(btScalar(4));

            segments[1] = btVector3(0, 1, 0);
            segments[1].setW(btScalar(4));

            segments[2] = btVector3(-1, 0, 0);
            segments[2].setW(btScalar(4));

            segments[3] = btVector3(0, 1, 0);
            segments[3].setW(btScalar(4));
            
            segments[4] = btVector3(1, 0, 0);
            segments[4].setW(btScalar(4.2));
            
            segments[5] = btVector3(0, -1, 0);
            segments[5].setW(btScalar(12));
            
            segments[6] = btVector3(-1, 0, 0);
            segments[6].setW(btScalar(4.2));
            
            segments[7] = btVector3(0, 1, 0);
            segments[7].setW(btScalar(4));
            return segments;
        }
  };
}
