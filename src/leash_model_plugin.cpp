#include <ros/ros.h>
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
      ROS_INFO("Creating Leash Model Plugin");
    }
    
    public: ~LeashModelPlugin() {
      ROS_INFO("Destroying Leash Model Plugin");
    }

    public: void Load(physics::ModelPtr _leash, sdf::ElementPtr /*_sdf*/) {
      ROS_INFO("Loading Leash Model Plugin");

      // Store the pointer to the world
      this->world = _leash->GetWorld();
      
      nh.param("leash_length", leashLength, 1.5);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&LeashModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate() {
        // Find the robot
        if(!robotHand){
            // Find the robots hand
            const physics::ModelPtr robot = this->world->GetModel("pr2");
            if(!robot){
                ROS_ERROR("Failed to locate the robot model");
                return;
            }
            robotHand = robot->GetLink("r_gripper_l_finger_tip_link");
            if(!robotHand){
                ROS_ERROR("Failed to loccate the robot finger tip link");
                return;
            }
        }
        
        if(!dogBody){
            // Find the dog.
            const physics::ModelPtr dog = this->world->GetModel("dog");
            if(!dog){
                ROS_ERROR("Could not locate dog model.");
                return;
            }

            dogBody = dog->GetLink("body");
            if(!dogBody){
                ROS_ERROR("Could not locate dog body link.");
                return;
            }
        }

      // Calculate the distance between the two.
      const math::Vector3 handPosition = robotHand->GetWorldPose().pos;
      const math::Vector3 dogPosition = dogBody->GetWorldPose().pos;
      const double distance = handPosition.Distance(dogPosition);

      // Now determine the ratio of force to apply using a sigmeud smoothing
      // function.
      // Octave function:
      // x = [0:0.01:2.5];
      // y = 1 ./ (1 + e.^(-24*(x - 2)));
      const double ratio = 1.0 / (1.0 + exp(-24.0 * (abs(distance) - leashLength)));

      // The hand force is a spring like attractive force between the hand and
      // the dog.
      // Determine the angle between the robot and the dog in the world frame.
      double a = atan2((handPosition.y - dogPosition.y), (handPosition.x - dogPosition.x));
      
      math::Vector3 handForce;
      handForce.x = SPRING_FORCE * cos(a);
      handForce.y = SPRING_FORCE * sin(a);
      handForce.z = 0;

      // Reduce the force.
      const math::Vector3 appliedForce = handForce * ratio;

      ROS_DEBUG("Applying force x: %f y: %f at angle %f with ratio: %f at distance: %f", appliedForce.x, appliedForce.y, a, ratio, distance);

      // Apply the force to the dog.
      dogBody->AddForce(appliedForce);
 
      // Apply the opposite force to the hand.
      // TODO: We should do this, but it can get the arm to a position where it can't move easily.
      // robotHand->AddForce(math::Vector3(-appliedForce.x, -appliedForce.y, 0.0));
    }
    
    // Pointer to the hand
    private: physics::LinkPtr robotHand;
    
    // Pointer to the dog
    private: physics::LinkPtr dogBody;
    
    // Pointer to the world
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Amount of force the leash can apply at its maximum
    private: static const double SPRING_FORCE = 800.0;
    
    double leashLength;
    
    ros::NodeHandle nh;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeashModelPlugin)
}
