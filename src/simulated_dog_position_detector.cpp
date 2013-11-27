#include <ros/ros.h>
#include <dogsim/utils.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/DogPosition.h>

namespace {
  using namespace std;
  using namespace dogsim;
  
  class SimulatedDogPositionDetector {
    private:
      //! Publisher for the dog positon
      ros::Publisher dogPositionPub;
      
      //! Frequency at which to search for the dog
      ros::Timer timer;
      
      //! The node handle we'll be using
      ros::NodeHandle nh;

      //! Private nh
      ros::NodeHandle pnh;

      //! Cached service client.
      ros::ServiceClient modelStateServ;
   public:
      //! ROS node initialization
      SimulatedDogPositionDetector():pnh("~"){
        dogPositionPub = nh.advertise<DogPosition>("/dog_position_detector/dog_position", 1);
        
        ros::service::waitForService("/gazebo/get_model_state");
        
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer = nh.createTimer(ros::Duration(0.1), &SimulatedDogPositionDetector::callback, this);
        timer.start();
      }

    private:
        geometry_msgs::PoseStamped getDogPose(){
            gazebo_msgs::GetModelState modelState;
            modelState.request.model_name = "dog";
            modelStateServ.call(modelState);
            geometry_msgs::PoseStamped dogPose;
            dogPose.header.stamp = ros::Time::now();
            dogPose.header.frame_id = "/map";
            dogPose.pose = modelState.response.pose;
            return dogPose;
        }
        
    void callback(const ros::TimerEvent& event){
        
        // Lookup the current position of the dog.
        geometry_msgs::PoseStamped dogPose = getDogPose();
        
        DogPosition dogPositionMsg;
        dogPositionMsg.pose = dogPose;
        dogPositionMsg.unknown = false;
        dogPositionMsg.measuredTime = event.current_real;
        // Publish the event
        ROS_DEBUG("Publishing a dog position event");
        
        dogPositionPub.publish(dogPositionMsg);
      }
  };
}

  int main(int argc, char** argv){
    ros::init(argc, argv, "simulated_dog_position_detector");

    SimulatedDogPositionDetector positionDetector;
    ros::spin();
    ROS_INFO("Exiting Simulated Dog Position Detector");
    return 0;
  }
