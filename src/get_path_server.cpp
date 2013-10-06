#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <geometry_msgs/Point.h>
#include "path_provider.h"
#include "lissajous_path_provider.h"
#include "rectangle_path_provider.h"
#include "block_walk_path_provider.h"
#include <dogsim/GetEntirePath.h>

namespace {
  using namespace ros;
  using namespace std;

  class GetPathServer {
    private:
      NodeHandle nh;
      NodeHandle pnh;
	  ros::ServiceServer service;
      ros::ServiceServer startService;
      ros::ServiceServer maxService;
      ros::ServiceServer entirePathService;
      bool started;
      double startTime;
      auto_ptr<PathProvider> pathProvider;
    public:
      
      GetPathServer() : pnh("~"), started(false), startTime(0){
        service = nh.advertiseService("/dogsim/get_path", &GetPathServer::getPath, this);
        entirePathService = nh.advertiseService("/dogsim/get_entire_path", &GetPathServer::getEntirePath, this);
        startService = nh.advertiseService("/dogsim/start", &GetPathServer::start, this);
        maxService = nh.advertiseService("/dogsim/maximum_time", &GetPathServer::maximumTime, this);
        
        string pathType;
        pnh.param<string>("path_type", pathType, "lissajous");
        if(pathType == "lissajous"){
            pathProvider.reset(new LissajousPathProvider());
        }
        else if(pathType == "rectangle"){
            pathProvider.reset(new RectanglePathProvider());
        }
        else if(pathType == "blockwalk"){
            pathProvider.reset(new BlockWalkPathProvider());
        }
        else {
            ROS_ERROR("Unknown path provider type: %s", pathType.c_str());
            return;
        }
        pathProvider->init();
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
          res.maximumTime = pathProvider->getMaximumTime();
          ROS_DEBUG("Returning maximum time: %f", res.maximumTime);
          return true;
      }
      
      bool getEntirePath(dogsim::GetEntirePath::Request& req, dogsim::GetEntirePath::Response& res){
          ROS_DEBUG("Getting entire path for max time %f and increment %f", pathProvider->getMaximumTime(), req.increment);
          for(double t = 0; t < pathProvider->getMaximumTime(); t += req.increment){
              res.points.push_back(pathProvider->positionAtTime(t));
          }
          return true;
      }
      
      bool getPath(dogsim::GetPath::Request& req, dogsim::GetPath::Response& res){
        res.elapsedTime = req.time - startTime;
        if(!started){
            // Not started yet.
            res.ended = false;
            res.started = false;
        }
        else if((req.time - startTime) > pathProvider->getMaximumTime()){
          res.ended = true;
          res.started = true;
        }
        else {
          res.started = true;
          res.ended = false;
        }
        
        res.point = pathProvider->positionAtTime((req.time - startTime));
        assert(res.point.header.frame_id.size() > 0);
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

