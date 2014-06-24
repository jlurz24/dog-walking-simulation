#pragma once
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

namespace {


  const ros::Duration SLOPE_DELTA(0.01);

  class PathProvider {
    public:
	    virtual ~PathProvider(){};
        virtual void init() = 0;
        virtual ros::Duration getMaximumTime() const = 0;

        geometry_msgs::PoseStamped poseAtTime(const ros::Duration baseT) const {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.pose.position = positionAtTime(baseT);

            geometry_msgs::PoseStamped goal2;
            goal2.header.frame_id = "map";
            goal2.pose.position = positionAtTime(baseT + SLOPE_DELTA);

            // Calculate the vector of the tangent line.
            tf2::Vector3 tangent = tf2::Vector3(goal2.pose.position.x, goal2.pose.position.y, 0)
                            - tf2::Vector3(goal.pose.position.x, goal.pose.position.y, 0);
            tangent.normalize();

            // Calculate the yaw so we can create an orientation.
            tfScalar yaw = tfAtan2(tangent.y(), tangent.x());
            goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            return goal;
        }
    protected:
        virtual geometry_msgs::Point positionAtTime(const ros::Duration baseT) const = 0;
  };
}

