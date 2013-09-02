#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/math/constants/constants.hpp>
#include <boost/random/uniform_real.hpp>
#include <stdlib.h>
#include <dogsim/utils.h>
#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <dogsim/GetDogPlannedPosition.h>

using namespace std;

namespace gazebo {
  static const double pi = boost::math::constants::pi<double>();

  class DogModelPlugin : public ModelPlugin {
    public: DogModelPlugin(){
        ROS_INFO("Creating Dog Plugin");
    }
    
    public: ~DogModelPlugin() {
      ROS_INFO("Destroying Dog Plugin");
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      ROS_INFO("Loading Dog Plugin");
      
      // Store the pointer to the model
      this->model = _parent;

      // Save the start time.
      this->previousTime = this->model->GetWorld()->GetSimTime();

      this->previousErrorX = this->previousErrorY = 0;
      this->forceX = this->forceY = 0.0;

      getPathClient = nh.serviceClient<dogsim::GetPath>("/dogsim/get_path", true);
      
      // Fetch the body link.
      body = this->model->GetLink("body");  
      
      ros::service::waitForService("/dogsim/get_path");
      ros::service::waitForService("/dogsim/start");
      ros::service::waitForService("/dogsim/maximum_time");
      
      // Advertise a service to get the dogs planned position.
      service = nh.advertiseService("/dogsim/dog_planned_position", &DogModelPlugin::getPlannedPosition, this);
      dogGoalVizPub = nh.advertise<visualization_msgs::Marker>("dogsim/dog_goal_viz", 1);
      
      // Initialize the gaussians.
      initGaussians();
      
      // Start the path if we are in solo mode. In regular mode the robot does this.
      bool isSoloDog;
      nh.param<bool>("solo_dog", isSoloDog, false);
      if(isSoloDog){
          startPath();
      }
      
      nh.param<double>("dog_kp", KP, KP_DEFAULT);
      nh.param<double>("dog_kd", KD, KD_DEFAULT);
      ROS_INFO("Dog PD operation with KP %f KD %f", KP, KD);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&DogModelPlugin::OnUpdate, this));
    }

    private: bool getPlannedPosition(dogsim::GetDogPlannedPosition::Request& req, dogsim::GetDogPlannedPosition::Response& res) {        bool running;
        const math::Vector3 plannedPosition = calcGoalPosition(common::Time(req.time.toSec()), running);
        res.point.header.stamp = req.time;
        res.point.header.frame_id = "/map";
        res.point.point.x = plannedPosition.x;
        res.point.point.y = plannedPosition.y;
        res.point.point.z = plannedPosition.z;
        return running;
    }
    
    private: void startPath(){
          dogsim::StartPath startPath;
          startPath.request.time = this->model->GetWorld()->GetSimTime().Double();
          ros::ServiceClient startPathClient = nh.serviceClient<dogsim::StartPath>("/dogsim/start", false);
          if(!startPathClient.call(startPath)){
              ROS_ERROR("Failed to start path");
          }
    }
    
    private: void initGaussians(){
        dogsim::MaximumTime maxTime;
        ros::ServiceClient maxTimeClient = nh.serviceClient<dogsim::MaximumTime>("/dogsim/maximum_time", false);
        if(!maxTimeClient.call(maxTime)){
            ROS_ERROR("Failed to call maximum time");
        }
        
        // Determine if we should start a new gaussian.
        boost::uniform_real<> randomZeroToOne(0, 1);
        
        ROS_INFO("Initializing gaussians. Maximum time is %f", maxTime.response.maximumTime);
        // Precompute all the lissajous cycles.
        for(double t = 0; t <= maxTime.response.maximumTime; t += 1.0 / SIMULATOR_CYCLES_PER_SECOND){
            if(randomZeroToOne(rng) < P_NEW_GAUSS / static_cast<double>(SIMULATOR_CYCLES_PER_SECOND)){
                // Create a structure with the parameters.
                GaussParams params;
                boost::uniform_real<> randomA(-pi, pi);
                params.a = randomA(rng);
       
                boost::uniform_real<> randomC(pi / 2, 8.0 * pi);
                params.c = randomC(rng);

                ROS_DEBUG("New gaussian created with a: %f c: %f at time %f", params.a, params.c, t);
                params.startTime = t;
                gaussParams.push_back(params);
            }
        }
        ROS_INFO("Completed initializing gaussians. Total gaussians is %lu", gaussParams.size());
    }
    
    // Called by the world update start event
    private: void OnUpdate() {

      // Calculate the desired position.
      common::Time currTime = this->model->GetWorld()->GetSimTime();
      bool running;
      math::Vector3 goalPosition = calcGoalPosition(currTime, running);
      if(!running){
          return;
      }
      
      // Publish the position.
      if(int(currTime.Double() * 1000) % 100 == 0 && dogGoalVizPub.getNumSubscribers() > 0){
          std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
          geometry_msgs::PointStamped goalPoint;
          goalPoint.point.x = goalPosition.x;
          goalPoint.point.y = goalPosition.y;
          goalPoint.point.z = goalPosition.z;
          goalPoint.header.stamp = ros::Time(currTime.Double());
          goalPoint.header.frame_id = "/map";
          dogGoalVizPub.publish(utils::createMarker(goalPoint.point, goalPoint.header, RED, true));
      }
      
      // Calculate current errors
      math::Vector3 worldPose = this->model->GetWorldPose().pos;
      double errorX = calcError(worldPose.x, goalPosition.x);
      double errorY = calcError(worldPose.y, goalPosition.y);
      common::Time deltat = currTime - this->previousTime;

      double errorDerivativeX = calcErrorDerivative(this->previousErrorX, errorX, deltat);
      double errorDerivativeY = calcErrorDerivative(this->previousErrorY, errorY, deltat);
      
      double outputX = calcPDOutput(errorX, errorDerivativeX);
      double outputY = calcPDOutput(errorY, errorDerivativeY);

      this->forceX += outputX;
      this->forceY += outputY;
 
      body->AddForce(math::Vector3(this->forceX, this->forceY, 0.0));

      // Save the current error for the next iteration.
      this->previousErrorX = errorX;
      this->previousErrorY = errorY;

      // Save the previous time.
      this->previousTime = currTime;
    }

    private: static double calcError(const double _currentPosition, const double _goalPosition) {
      return _goalPosition - _currentPosition;
    }

    private: static double calcErrorDerivative(const double _previousError, const double _currentError, const common::Time& _deltaTime) {
      // Check for initial condition.
      if(_previousError == 0.0){
        return 1.0;
      }
      return (_currentError - _previousError) / _deltaTime.Double();
    }

    private: double calcPDOutput(const double _error, const double _errorDerivative){
      return KP * _error + KD * _errorDerivative;
    }

    private: math::Vector3 calcGoalPosition(common::Time time, bool& running){
        dogsim::GetPath getPath;
        getPath.request.time = time.Double();
        getPathClient.call(getPath);
        if(!getPath.response.started || getPath.response.ended){
            running = false;
            return math::Vector3();
        }
        
        running = true;
        
        // Check the goal for the current time.
        gazebo::math::Vector3 base;
        base.x = getPath.response.point.point.x;
        base.y = getPath.response.point.point.y;
        base.z = getPath.response.point.point.z;  

        // Gaussian function is tuned for input = [1:700]
        math::Vector3 result = addGaussians(base, this->previousBase, getPath.response.elapsedTime);
        this->previousBase = base;
        return result;
    }

    private: math::Vector3 addGaussians(const math::Vector3& base, const math::Vector3& previousBase, const double t){

      // Start with the base vector
      math::Vector3 result = base;

      // Calculate the derivative
      double dx = 0;
      double dy = 0;
      if(t > 0){
        dx = base.x - previousBase.x;
        dy = base.y - previousBase.y;
      }

      // Calculate the normal
      double rx = dy;
      double ry = -dx;

      // Calculate the unit vector.
      double rl = sqrt(utils::square(rx) + utils::square(ry));
      double rux;
      double ruy;
      if(rl > 0){
        rux = rx / rl;
        ruy = ry / rl;
      }
      else {
        rux = rx;
        ruy = ry;
      }

      // Iterate over all gaussians.
      for(unsigned int i = 0; i < gaussParams.size(); ++i){
          // Start times are strictly increasing.
          if(t < gaussParams[i].startTime){
              break;
          }
          
          double gx = t - gaussParams[i].startTime - GAUSS_HALF_WIDTH * gaussParams[i].c;
          double gy = gaussParams[i].a * exp(-(utils::square(gx) / (2 * utils::square(gaussParams[i].c))));

          // Calculate the normal vector.
          double nx = rux * gy;
          double ny = ruy * gy;

          // Add it to the result.
          result.x += nx;
          result.y += ny;
      }
      return result;
    }

    //! ROS node handle
    private: ros::NodeHandle nh;
    
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Previous iteration's time
    private: common::Time previousTime;
    
    // Previous iteration's error in X
    private: double previousErrorX;

    // Previous iteration's error in Y
    private: double previousErrorY;

    // X Force
    private: double forceX;

    // Y Force
    private: double forceY;

    // Previous goal
    private: math::Vector3 previousBase;

    private: struct GaussParams {
      double a;
      double c;
      double startTime;
    };

    private: ros::ServiceClient getPathClient;
    
    private: ros::ServiceServer service;
    
    private: physics::LinkPtr body;
    
    // Parameters for all operating gaussians
    private: vector<GaussParams> gaussParams;

    // Pseudo random number generator. This will always use the same seed
    // so that every run is the same.
    private: boost::mt19937 rng;

    private: ros::Publisher dogGoalVizPub;
    
    // Number of simulator iterations per second.
    private: static const unsigned int SIMULATOR_CYCLES_PER_SECOND = 1000;

    // KP term
    private: double KP;

    private: static const double KP_DEFAULT = 0.6;
    
    // KD term
    private: double KD;

    private: static const double KD_DEFAULT = 0.35;
    
    // Probability of starting a new gaussian in each second
    private: static const double P_NEW_GAUSS = 0.16;

    // Multiple of sigma that captures nearly half of a gaussians width.
    private: static const double GAUSS_HALF_WIDTH = 3;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DogModelPlugin)
}
