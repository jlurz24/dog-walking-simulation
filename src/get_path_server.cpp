#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include "lissajous_path_provider.h"
#include "rectangle_path_provider.h"

namespace {
  using namespace ros;
  using namespace std;

  //! Factor to slow down the lissajous calculation.
  // TODO: Determine if this is the appropriate speed.
  static const double TIMESCALE_FACTOR = 100.0;

  //! Amount of time it takes to perform a full lissajous cycle.
  //  Note that this amount is slightly longer than the lissajous
  //  cycle time because the robot trails the goal point.
  static const double FULL_CYCLE_T = 4.85;

  class GetPathServer {
    private:
      NodeHandle nh;
      NodeHandle pnh;
	  ros::ServiceServer service;
      ros::ServiceServer startService;
      ros::ServiceServer maxService;
      bool started;
      bool ended;
      double startTime;
      auto_ptr<PathProvider> pathProvider;
    public:
      
      GetPathServer() : pnh("~"), started(false), ended(false), startTime(0){
        service = nh.advertiseService("/dogsim/get_path", &GetPathServer::getPath, this);
        startService = nh.advertiseService("/dogsim/start", &GetPathServer::start, this);
        maxService = nh.advertiseService("/dogsim/maximum_time", &GetPathServer::maximumTime, this);
        
        string pathType;
        pnh.param<string>("path_type", pathType, "lissajous");
        if(pathType == "lissajous"){
            pathProvider.reset(new LissajousPathProvider(FULL_CYCLE_T));
        }
        else if(pathType == "rectangle"){
            pathProvider.reset(new RectanglePathProvider(FULL_CYCLE_T));
        }
        else {
            ROS_ERROR("Unknown path provider type: %s", pathType.c_str());
        }
      }
      
    private:
      bool start(dogsim::StartPath::Request& req, dogsim::StartPath::Response& res){
          assert(!started);
          started = true;
          startTime = req.time;
          ROS_DEBUG("Starting path @ time: %f", startTime);
          return true;
      }
      
      bool maximumTime(dogsim::MaximumTime::Request& req, dogsim::MaximumTime::Response& res){
          res.maximumTime = FULL_CYCLE_T * TIMESCALE_FACTOR;
          ROS_DEBUG("Returning maximum time: %f", res.maximumTime);
          return true;
      }
      bool getPath(dogsim::GetPath::Request& req, dogsim::GetPath::Response& res){
	    
        res.elapsedTime = req.time - startTime;
        
        if(ended){
            res.ended = true;
            res.started = true;
            return true;
        }

        if(!started){
            // Not started yet.
            res.ended = false;
            res.started = false;
            return true;
        }
        
        if((req.time - startTime) / TIMESCALE_FACTOR > FULL_CYCLE_T){
          ROS_INFO("Setting end flag @ time %f", req.time);
          res.ended = ended = true;
          res.started = true;
          return true;
        }

        // TODO: Be smarter about making time scale based on velocity.
        res.started = true;
        res.ended = false;
        res.point = pathProvider->positionAtTime((req.time - startTime) / TIMESCALE_FACTOR);
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

