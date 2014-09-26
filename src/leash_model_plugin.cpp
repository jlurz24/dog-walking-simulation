#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/math/constants/constants.hpp>
#include <stdlib.h>
#include <time.h>
#include <dogsim/LeashInfo.h>



namespace {
using namespace std;
using namespace gazebo;

// Amount of force the leash can apply at its maximum
const double SPRING_FORCE = 60.0;

// Rate to run the updates to the force
const double UPDATE_RATE = 0.01;

class LeashModelPlugin : public ModelPlugin {
public:
    LeashModelPlugin() {
        ROS_INFO("Creating Leash Model Plugin");
        leashInfoPub = nh.advertise<dogsim::LeashInfo>("leash_model/info", 1);
    }

    ~LeashModelPlugin() {
        ROS_INFO("Destroying Leash Model Plugin");
    }

    void Load(physics::ModelPtr _leash, sdf::ElementPtr /*_sdf*/) {
        ROS_INFO("Loading Leash Model Plugin");

        // Store the pointer to the world
        this->world = _leash->GetWorld();

        nh.param("leash_length", leashLength, 1.5);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&LeashModelPlugin::OnUpdate, this));
        this->previousTime = this->world->GetSimTime();
    }

private:
    // Called by the world update start event
    void OnUpdate() {
        // Find the robot
        if(!robotHand){
            // Find the robots hand
            const physics::ModelPtr robot = this->world->GetModel("pr2");
            if(!robot){
                ROS_ERROR("Failed to locate the robot model");
                return;
            }
            robotHand = robot->GetLink("r_wrist_roll_link");
            if(!robotHand){
                ROS_ERROR("Failed to locate r_wrist_roll_link");
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

        common::Time currTime = this->world->GetSimTime();
        if (currTime.Double() - this->previousTime.Double() > UPDATE_RATE) {
            // Calculate the distance between the two.
            const math::Vector3 handPosition = robotHand->GetWorldPose().pos;
            const math::Vector3 dogPosition = dogBody->GetWorldPose().pos;
            const double distance = handPosition.Distance(dogPosition);

            // Now determine the ratio of force to apply using a sigmeud smoothing
            // function.
            // Octave function:
            // x = [0:0.01:2.5];
            // y = 1 ./ (1 + e.^(-48*(x - 1.25)));
            const double ratio = 1.0 / (1.0 + exp(-48.0 * (abs(distance) - leashLength)));

            // The hand force is a spring like attractive force between the hand and
            // the dog.
            const math::Vector3 handForce = (handPosition - dogPosition).Normalize();

            // Reduce the force.
            appliedForce = SPRING_FORCE * handForce * ratio;

            ROS_DEBUG("Applying force x: %f y: %f z: %f with ratio: %f at distance: %f", appliedForce.x, appliedForce.y, appliedForce.z, ratio, distance);

            if(leashInfoPub.getNumSubscribers() > 0){
                dogsim::LeashInfo info;
                info.header.stamp = ros::Time(currTime.Double());
                info.header.frame_id = "/map";
                info.force.x = appliedForce.x;
                info.force.y = appliedForce.y;
                info.force.z = appliedForce.z;
                info.distance = distance;
                info.ratio = ratio;
                leashInfoPub.publish(info);
            }
            this->previousTime = currTime;
        }
        // Apply the force to the dog.
        dogBody->AddForce(appliedForce);

        // Don't allow the extra spring force to be applied to the arm.
        // Apply the opposite force to the hand.
        robotHand->AddForce(math::Vector3(-appliedForce.x, -appliedForce.y, -appliedForce.z));

    }

    // Previous update time
    common::Time previousTime;
    
    // Pointer to the hand
    physics::LinkPtr robotHand;

    // Pointer to the dog
    physics::LinkPtr dogBody;

    // Pointer to the world
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    math::Vector3 appliedForce;

    double leashLength;

    ros::NodeHandle nh;

    ros::Publisher leashInfoPub;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LeashModelPlugin);
}
