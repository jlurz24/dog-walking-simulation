#pragma once
#include <geometry_msgs/Point.h>

namespace {
  class PathProvider {
      public:
        virtual geometry_msgs::PointStamped positionAtTime(const double t) const = 0;
  };
}

