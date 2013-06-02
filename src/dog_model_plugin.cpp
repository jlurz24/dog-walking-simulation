#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/math/constants/constants.hpp>
#include <stdlib.h>
#include <time.h>


using namespace std;

namespace gazebo {
  class DogModelPlugin : public ModelPlugin {
    public: DogModelPlugin() {
      cout << "Creating Dog Plugin" << endl;

      // Initialize random number generator.
      srand(time(NULL));
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

    private: static math::Vector3 calcGoalPosition(const double t){
      // Lissajous parameters.
      static const double a = sqrt(2);
      static const double DELTA = boost::math::constants::pi<long double>() / 2.0;
      static const double A = 5.0;
      static const double B = 1.5;
      static const double b = 2 * a;
      static const double SLOWING_FACTOR = 250.0;
      static const double GAUSS_MEAN = 0.0;
      static const double GAUSS_VARIANCE = 0.025;

      math::Vector3 goal;
      goal.z = 0.0;
      goal.x = A * sin(a * t / SLOWING_FACTOR + DELTA);
      goal.y = B * sin(b * t / SLOWING_FACTOR);

      goal.x += gaussianNoise(GAUSS_MEAN, GAUSS_VARIANCE);
      goal.y += gaussianNoise(GAUSS_MEAN, GAUSS_VARIANCE);
      return goal;
    }

    private: static double gaussianNoise(const double mean, const double variance){
      const unsigned int N = 20;
      double X = 0.0;
      
      for(unsigned int i = 0; i < N; ++i){
        X += rand() / double(RAND_MAX);
      }
      X = X - N / 2.0;
      X = X * sqrt(12.0 / N);
      X = mean + sqrt(variance) * X;
      return X;
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

    // KP term
    private: static const double KP = 0.05;

    // KD term
    private: static const double KD = 0.0125;

    // Max force
    private: static const double MAX_FORCE = 50.0;

    // Amount of time to wait before starting movement.
    private: static const double WAIT_TIME = 30.0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DogModelPlugin)
}
