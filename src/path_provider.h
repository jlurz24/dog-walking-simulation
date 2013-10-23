#pragma once
#include <geometry_msgs/Point.h>

namespace {
  class PathProvider {
    public:
	    virtual ~PathProvider() = 0;
        virtual void init() = 0;
        virtual geometry_msgs::PointStamped positionAtTime(const double t) const = 0;
        virtual double getMaximumTime() const = 0;
  };
}

