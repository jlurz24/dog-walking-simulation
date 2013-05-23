#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

namespace gazebo {
  class DogModelPlugin : public ModelPlugin {
    public: DogModelPlugin() {
      std::cout << "Creating Dog Plugin" << std::endl;
    }
    
    public: ~DogModelPlugin() {
      std::cout << "Destroying Dog Plugin" << std::endl;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      std::cout << "Loading plugin" << std::endl;

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&DogModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate() {
      // Fetch the body link.
      physics::LinkPtr body = model->GetLink("body");

      // Wait until the object hits the ground.
      if(model->GetWorldPose().pos.z < 0.1){
        body->AddForce(math::Vector3(1, 0., 0.5));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DogModelPlugin)
}
