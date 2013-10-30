#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <dogsim/GetPlannedRobotPose.h>
#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include "lissajous_path_provider.h"
#include "rectangle_path_provider.h"
#include "block_walk_path_provider.h"
#include <dogsim/GetEntirePath.h>
#include <tf/transform_listener.h>

namespace {
using namespace ros;
using namespace std;

//! Slope delta for calculating robot path.
static const ros::Duration SLOPE_DELTA(0.01);

static const double TRAILING_DISTANCE = 0;

//! Shift distance from base to desired arm position
//! Calculated as the negative of /base_footprint to /r_wrist_roll_link in x axis
static const double SHIFT_DISTANCE = 0.6;

class GetPathServer {

private:
	NodeHandle nh;
	NodeHandle pnh;
	ros::ServiceServer service;
	ros::ServiceServer startService;
	ros::ServiceServer maxService;
	ros::ServiceServer entirePathService;
	ros::ServiceServer plannedRobotPoseService;

	bool started;
	ros::Time startTime;
	auto_ptr<PathProvider> pathProvider;

public:

	GetPathServer() :
		pnh("~"), started(false) {
		service = nh.advertiseService("/dogsim/get_path",
				&GetPathServer::getPath, this);
		entirePathService = nh.advertiseService("/dogsim/get_entire_path",
				&GetPathServer::getEntirePath, this);
		startService = nh.advertiseService("/dogsim/start",
				&GetPathServer::start, this);
		maxService = nh.advertiseService("/dogsim/maximum_time",
				&GetPathServer::maximumTime, this);
		plannedRobotPoseService = nh.advertiseService(
				"/dogsim/get_planned_robot_pose",
				&GetPathServer::getPlannedRobotPose, this);

		string pathType;
		pnh.param<string>("path_type", pathType, "lissajous");
		if (pathType == "lissajous") {
			pathProvider.reset(new LissajousPathProvider());
		} else if (pathType == "rectangle") {
			pathProvider.reset(new RectanglePathProvider());
		} else if (pathType == "blockwalk") {
			pathProvider.reset(new BlockWalkPathProvider());
		} else {
			ROS_ERROR("Unknown path provider type: %s", pathType.c_str());
			return;
		}
		ROS_INFO("%s path type selected", pathType.c_str());
		pathProvider->init();
	}

private:
	bool start(dogsim::StartPath::Request& req,
			dogsim::StartPath::Response& res) {
		assert(!started);
		started = true;
		startTime = req.time;
		ROS_DEBUG("Starting path @ time: %f", startTime.toSec());
		return true;
	}

	bool maximumTime(dogsim::MaximumTime::Request& req,
			dogsim::MaximumTime::Response& res) {
		res.maximumTime = pathProvider->getMaximumTime();
		ROS_DEBUG("Returning maximum time: %f", res.maximumTime.toSec());
		return true;
	}

	bool getEntirePath(dogsim::GetEntirePath::Request& req,
			dogsim::GetEntirePath::Response& res) {
		ROS_DEBUG(
				"Getting entire path for max time %f and increment %f", pathProvider->getMaximumTime().toSec(), req.increment);
		for (double t = 0; t < pathProvider->getMaximumTime().toSec();
				t += req.increment) {
			res.points.push_back(pathProvider->positionAtTime(ros::Duration(t)));
		}
		return true;
	}

	bool getPlannedRobotPose(dogsim::GetPlannedRobotPose::Request& req,
			dogsim::GetPlannedRobotPose::Response& res) {

		computeStartAndEnd(req.time, res.started, res.ended);
		if (!res.started || res.ended) {
			return true;
		}

		const geometry_msgs::PointStamped dogGoal =
				pathProvider->positionAtTime(req.time - startTime);

		const geometry_msgs::PointStamped goal2 = pathProvider->positionAtTime((req.time - startTime) + SLOPE_DELTA);

		// Calculate the vector of the tangent line.
		btVector3 tangent = btVector3(goal2.point.x, goal2.point.y, 0)
						- btVector3(dogGoal.point.x, dogGoal.point.y, 0);
		tangent.normalize();

		// Now select a point on the vector but slightly behind.
		btVector3 backGoal = btVector3(dogGoal.point.x, dogGoal.point.y, 0)
						- tangent * btScalar(TRAILING_DISTANCE);

		// Rotate the vector to perpendicular
		btVector3 perp = tangent.rotate(btVector3(0, 0, 1),
				btScalar(boost::math::constants::pi<double>() / 2.0));

		// Select a point on the perpendicular line.
		btVector3 finalGoal = backGoal + perp * btScalar(SHIFT_DISTANCE);

		geometry_msgs::Point robotGoal;
		robotGoal.x = finalGoal.x();
		robotGoal.y = finalGoal.y();
		robotGoal.z = finalGoal.z();

		res.pose.pose.position = robotGoal;
		res.pose.header = dogGoal.header;

		// Calculate the yaw so we can create an orientation.
		btScalar yaw = btAtan2(tangent.y(), tangent.x());
		res.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

		return true;
	}

	bool getPath(dogsim::GetPath::Request& req,
			dogsim::GetPath::Response& res) {
		res.elapsedTime = req.time - startTime;
		computeStartAndEnd(ros::Time(req.time), res.started, res.ended);
		res.point = pathProvider->positionAtTime(req.time - startTime);
		assert(res.point.header.frame_id.size() > 0);
		return true;
	}

	void computeStartAndEnd(const ros::Time& time, uint8_t& rStarted,
			uint8_t& rEnded) const {
		if (!this->started) {
			// Not started yet.
			rEnded = false;
			rStarted = false;
		} else if ((time - startTime) > pathProvider->getMaximumTime()) {
			rEnded = true;
			rStarted = true;
		} else {
			rStarted = true;
			rEnded = false;
		}
	}
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "get_path");
	GetPathServer getPathServer;
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	return 0;
}

