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
  class LeashModelPlugin : public ModelPlugin {
    public: LeashModelPlugin() {
      cout << "Creating Leash Model Plugin" << endl;
    }
    
    public: ~LeashModelPlugin() {
      cout << "Destroying Leash Model Plugin" << endl;
    }

    public: void Load(physics::ModelPtr _leash, sdf::ElementPtr /*_sdf*/) {
      cout << "Loading Leash Model Plugin" << endl;

      // Store the pointer to the world
      this->world = _leash->GetWorld();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&LeashModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate() {
      // Find the robots hand
      const physics::ModelPtr robot = this->world->GetModel("pr2");
      if(!robot){
        cout << "Could not locate robot." << endl;
        return;
      }
      const physics::LinkPtr robotHand = robot->GetLink("r_gripper_l_finger_tip_link");
      if(!robotHand){
        cout << "Could not locate robot hand link." << endl;
        return;
      }

      // Find the dog.
      const physics::ModelPtr dog = this->world->GetModel("dog");
      if(!dog){
        cout << "Could not locate dog model." << endl;
        return;
      }
      const physics::LinkPtr dogBody = dog->GetLink("body");
      if(!dogBody){
        cout << "Could not locate dog body link." << endl;
        return;
      }

      // Calculate the distance between the two.
      const math::Vector3 handPosition = robotHand->GetWorldPose().pos;
      const math::Vector3 dogPosition = dogBody->GetWorldPose().pos;
      const double distance = handPosition.Distance(dogPosition);

      // Now determine the ratio of force to apply using a sigmeud smoothing
      // function.
      const double ratio = 1.0 / (1.0 + exp(-12.0 * (abs(distance) - LEASH_LENGTH)));

      // The hand force is a spring like attractive force between the hand and
      // the dog.
      // Determine the angle between the robot and the dog in the world frame.
      double a = atan((dogPosition.y - handPosition.y) / (dogPosition.x - handPosition.x));
      
      // Correct the direction of the angle.
      if(distance < 0){
        a += boost::math::constants::pi<double>();
      }

      math::Vector3 handForce;
      handForce.x = SPRING_FORCE * cos(a);
      handForce.y = SPRING_FORCE * sin(a);
      handForce.z = 0;

      // Reduce the force.
      const math::Vector3 appliedForce = handForce * ratio;

      // Apply the force to the dog.
      dogBody->AddForce(appliedForce);
 
      // Apply the opposite force to the hand.
      robotHand->AddForce(math::Vector3(-appliedForce.x, -appliedForce.y, 0.0));
    }
    
    // Pointer to the world
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Length of the leash
    private: static const double LEASH_LENGTH = 15.0;

    // Amount of force the leash can apply at its maximum
    private: static const double SPRING_FORCE = 50.0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeashModelPlugin)
}
