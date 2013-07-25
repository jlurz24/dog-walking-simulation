#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/Point.h>

namespace {
  using namespace ros;
  using namespace std;

  //! Factor to slow down the lissajous calculation.
  static const double TIMESCALE_FACTOR = 10.0;

  //! Amount of time it takes to perform a full lissajous cycle.
  //  Note that this amount is slightly longer than the lissajous
  //  cycle time because the robot trails the goal point.
  static const double LISSAJOUS_FULL_CYCLE_T = 4.85;

  class GetPathServer {
    private:
      NodeHandle nh;
	  ros::ServiceServer service;
      bool started;
      bool ended;
      double startTime;
    public:
      GetPathServer():started(false), ended(false), startTime(0){
        service = nh.advertiseService("/dogsim/get_path", &GetPathServer::getPath, this);
      }
    private:
      geometry_msgs::PointStamped lissajous(const double t){

        // Lissajous parameters.
        static const double a = sqrt(2);
        static const double delta = boost::math::constants::pi<long double>() / 2.0;
        static const double A = 5.0;
        static const double B = 1.5;
        static const double b = 2 * a;

        geometry_msgs::PointStamped goal;
        goal.header.frame_id = "/map";
        goal.point.x = -(A * sin(a * t + delta)) + 6.5; // Offset the start and invert;
        goal.point.y = B * sin(b * t);
        goal.point.z = 0.0;
        return goal;
      }

      bool getPath(dogsim::GetPath::Request& req, dogsim::GetPath::Response& res){
	    res.elapsedTime = req.time - startTime;
        if(ended){
          res.ended = true;
          res.started = true;
          return true;
        }

        if(!started){
          if(!req.start){
            // Not started yet.
            res.ended = false;
            res.started = false;
            return true;
          }

          // First call. Initialize.
          started = true;
          startTime = req.time;
		  ROS_INFO("Starting path @ time: %f", startTime);
        }
        else if((req.time - startTime) / TIMESCALE_FACTOR > LISSAJOUS_FULL_CYCLE_T){
          ROS_INFO("Setting end flag @ time %f", req.time);
          res.ended = ended = true;
          res.started = true;
          return true;
        }

        // TODO: Be smarter about making time scale based on velocity.
        res.started = true;
        res.ended = false;
        res.point = lissajous((req.time - startTime) / TIMESCALE_FACTOR);
		res.point.header.stamp = Time(req.time);
        return true;
      }
  };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "get_path");
  GetPathServer getPathServer;
  ros::spin();
  return 0;
}

