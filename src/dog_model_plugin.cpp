#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/math/constants/constants.hpp>
#include <boost/random/uniform_real.hpp>
#include <stdlib.h>
#include <dogsim/utils.h>

using namespace std;

namespace gazebo {
  static const double pi = boost::math::constants::pi<double>();

  class DogModelPlugin : public ModelPlugin {
    public: DogModelPlugin() {
      cout << "Creating Dog Plugin" << endl;
    }
    
    public: ~DogModelPlugin() {
      cout << "Destroying Dog Plugin" << endl;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      cout << "Loading plugin" << endl;

      // Store the pointer to the model
      this->model = _parent;

      // Save the start time.
      this->previousTime = this->model->GetWorld()->GetSimTime();

      this->previousErrorX = this->previousErrorY = 0;
      this->forceX = this->forceY = 0.0;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&DogModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate() {
      // Give the simulator a few seconds to start up.
      if(this->model->GetWorld()->GetSimTime().Double() < WAIT_TIME){
        // Save the previous time.
        this->previousTime = this->model->GetWorld()->GetSimTime();
        return;
      }
   
      // Fetch the body link.
      physics::LinkPtr body = this->model->GetLink("body");        
       
        // Calculate the desired position. 
        math::Vector3 goalPosition = calcGoalPosition(this->model->GetWorld()->GetSimTime().Double() - WAIT_TIME);
        // Calculate current errors.
        double errorX = calcError(this->model->GetWorldPose().pos.x, goalPosition.x);
        double errorY = calcError(this->model->GetWorldPose().pos.y, goalPosition.y);

        common::Time deltat = this->model->GetWorld()->GetSimTime() - this->previousTime;

        double errorDerivativeX = calcErrorDerivative(this->previousErrorX, errorX, deltat);
        double errorDerivativeY = calcErrorDerivative(this->previousErrorY, errorY, deltat);

        double outputX = calcPDOutput(errorX, errorDerivativeX);
        double outputY = calcPDOutput(errorY, errorDerivativeY);

        this->forceX += outputX;
        this->forceY += outputY;
 
        // Prevent excessive force.
        this->forceX = this->forceX > 0 ? min(this->forceX, MAX_FORCE) : max(this->forceX, -MAX_FORCE);
        this->forceY = this->forceY > 0 ? min(this->forceY, MAX_FORCE) : max(this->forceY, -MAX_FORCE);
        body->AddForce(math::Vector3(this->forceX, this->forceY, 0.0));

        // Save the current error for the next iteration.
        this->previousErrorX = errorX;
        this->previousErrorY = errorY;

        // Save the previous time.
        this->previousTime = this->model->GetWorld()->GetSimTime();
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

    private: static double calcPDOutput(const double _error, const double _errorDerivative){
      return KP * _error + KD * _errorDerivative;
    }

    private: math::Vector3 calcGoalPosition(const double t){
      // Slow the pathway down
      double scaledT = t / TIME_SCALE_FACTOR;

      math::Vector3 base = utils::lissajous(scaledT);
      math::Vector3 result = addGaussians(base, this->previousBase, scaledT);
      this->previousBase = base;
      return result;
    }

    private: math::Vector3 addGaussians(const math::Vector3& base, const math::Vector3& previousBase, const double t){
      
      // Start with the base vector      
      math::Vector3 result = base;

      // Determine if we should start a new gaussian.
      boost::uniform_real<> randomZeroToOne(0,1);
      
      if(randomZeroToOne(rng) < P_NEW_GAUSS / static_cast<double>(SIMULATOR_CYCLES_PER_SECOND)){
        // Create a structure with the parameters.
        GaussParams params;
        boost::uniform_real<> randomA(-pi/4, pi/4);
        params.a = randomA(rng);

        boost::uniform_real<> randomC(pi / 2, 2 * pi);
        params.c = randomC(rng);

        params.startTime = t;
        gaussParams.push_back(params);
      }

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
      double rl = sqrt(pow(rx, 2) + pow(ry, 2));
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
        double gx = t - gaussParams[i].startTime - GAUSS_HALF_WIDTH * gaussParams[i].c;
        double gy = gaussParams[i].a * exp(-pow(gx, 2) / (2 * pow(gaussParams[i].a, 2)));

        // Calculate the normal vector.
        double nx = rux * gy;
        double ny = ruy * gy;

        // Add it to the result.
        result.x += nx;
        result.y += ny;
      }

      return result;
    }

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

    // Parameters for all operating gaussians
    private: vector<GaussParams> gaussParams;

    // Pseudo random number generator.
    private: boost::mt19937 rng;

    // Number of simulator iterations per second.
    private: static const unsigned int SIMULATOR_CYCLES_PER_SECOND = 1000;

    // KP term
    private: static const double KP = 0.005;

    // KD term
    private: static const double KD = 0.025;

    // Max force
    private: static const double MAX_FORCE = 50.0;

    // Amount of time to wait before starting movement.
    private: static const double WAIT_TIME = 5.0;

    // Scaling factor to slow growth of t.
    private: static const double TIME_SCALE_FACTOR = 5.0;
    
    // Probability of starting a new gaussian in a given second.
    private: static const double P_NEW_GAUSS = 0.80;

    // Multiple of sigma that captures nearly half of a gaussians width.
    private: static const double GAUSS_HALF_WIDTH = 3;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DogModelPlugin)
}
