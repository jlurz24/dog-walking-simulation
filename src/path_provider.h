#pragma once
#include <geometry_msgs/Point.h>

namespace {
  class PathProvider {
    public:
	    virtual ~PathProvider(){};
        virtual void init() = 0;
        virtual geometry_msgs::PointStamped positionAtTime(const ros::Duration t) const = 0;
        virtual ros::Duration getMaximumTime() const = 0;
  };
}

