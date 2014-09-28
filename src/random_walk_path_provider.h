#include "path_provider.h"
#include "piecewise_linear_path_provider.h"
#include <vector>
#include <tf2/LinearMath/btVector3.h>

namespace {
  
  class RandomWalkPathProvider : public PiecewiseLinearPathProvider {
      public:
          RandomWalkPathProvider(){}
          virtual ~RandomWalkPathProvider(){}
      protected:
        virtual std::vector<btVector3> getSegments() const {
            std::vector<btVector3> segments(16);
            unsigned int i = 0;
            segments[i] = btVector3(1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(-1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, -1, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(-1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(-1, 0, 0);
            segments[i++].setW(btScalar(0.5));

            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));
            
            segments[i] = btVector3(1, 0, 0);
            segments[i++].setW(btScalar(0.5));
            
            segments[i] = btVector3(0, -1, 0);
            segments[i++].setW(btScalar(0.5));
            
            segments[i] = btVector3(-1, 0, 0);
            segments[i++].setW(btScalar(0.5));
            
            segments[i] = btVector3(0, 1, 0);
            segments[i++].setW(btScalar(0.5));
            return segments;
        }
  };
}
