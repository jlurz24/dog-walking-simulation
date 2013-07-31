#include <ros/ros.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/DogPosition.h>

namespace {
  using namespace std;
  
  class DogPositionDetector {
    private:
      //! Publisher for the dog position visualization.
      ros::Publisher dogVizPub_;

      //! Publisher for the dog positon
      ros::Publisher dogPositionPub_;

      //! Frequency at which to search for the dog
      ros::Timer timer_;
      
      //! The node handle we'll be using
      ros::NodeHandle nh_;

      //! Private nh
      ros::NodeHandle pnh_;

      //! We will be listening to TF transforms
      tf::TransformListener tf_;

   public:
      //! ROS node initialization
      DogPositionDetector():pnh_("~"){
        dogVizPub_ = nh_.advertise<visualization_msgs::Marker>("/dog_position_viz", 1);
        dogPositionPub_ = nh_.advertise<dogsim::DogPosition>("/dog_position", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        timer_ = nh_.createTimer(ros::Duration(0.1), &DogPositionDetector::callback, this);
        timer_.start();
      }

   private:
      void callback(const ros::TimerEvent& event){
        // Lookup the current position.
        if(!tf_.waitForTransform("/base_footprint", "/map", event.current_real, ros::Duration(0.25))){
          ROS_WARN("Failed to get transform. Aborting iteration.");
          return;
        }
        
        geometry_msgs::PoseStamped dogPose;

        // Lookup the current position of the dog.
        ros::ServiceClient modelStateServ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "dog";
        modelStateServ.call(modelState);
        dogPose.header.stamp = event.current_real;
        dogPose.header.frame_id = "/map";
        dogPose.pose = modelState.response.pose;
    
        // Visualize the dog.
        std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
        dogVizPub_.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));

        geometry_msgs::PoseStamped dogInBaseFrame;

        // Determine the relative dog position
        tf_.transformPose("/base_footprint", dogPose, dogInBaseFrame);
        dogInBaseFrame.header.frame_id = "/base_footprint";
        dogInBaseFrame.header.stamp = event.current_real;

        // Publish the event
        ROS_DEBUG("Publishing a dog position event");
        dogsim::DogPosition dogPositionMsg;
        dogPositionMsg.pose = dogInBaseFrame;
        dogPositionMsg.header = dogInBaseFrame.header;
        dogPositionPub_.publish(dogPositionMsg);
      }
  };
}

  int main(int argc, char** argv){
    ros::init(argc, argv, "dog_position_detector");

    DogPositionDetector positionDetector;
    ros::spin();
    ROS_INFO("Exiting Dog Position Detector");
    return 0;
  }
