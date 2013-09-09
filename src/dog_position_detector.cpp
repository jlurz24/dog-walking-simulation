#include <ros/ros.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetDogPlannedPosition.h>

namespace {
  using namespace std;
  using namespace dogsim;
  
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

      //! Cached service client.
      ros::ServiceClient modelStateServ;
      
   public:
      //! ROS node initialization
      DogPositionDetector():pnh_("~"){
        // TODO: Move these to a scope
        dogVizPub_ = nh_.advertise<visualization_msgs::Marker>("/dog_position_viz", 1);
        dogPositionPub_ = nh_.advertise<DogPosition>("/dog_position", 1);
        
        ros::service::waitForService("/gazebo/get_model_state");
        
        modelStateServ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer_ = nh_.createTimer(ros::Duration(0.1), &DogPositionDetector::callback, this);
        timer_.start();
      }

    private:
   
        geometry_msgs::PoseStamped getDogPose(const ros::Time& time){
            gazebo_msgs::GetModelState modelState;
            modelState.request.model_name = "dog";
            modelStateServ.call(modelState);
            geometry_msgs::PoseStamped dogPose;
            dogPose.header.stamp = time;
            dogPose.header.frame_id = "/map";
            dogPose.pose = modelState.response.pose;
            return dogPose;
        }
        
    void callback(const ros::TimerEvent& event){
        
        // Lookup the current position of the dog.
        geometry_msgs::PoseStamped dogPose = getDogPose(event.current_real);
        
        // Visualize the dog.
        if(dogVizPub_.getNumSubscribers() > 0){
            std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
            dogVizPub_.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));
        }
        DogPosition dogPositionMsg;
        dogPositionMsg.pose = dogPose;
        
        // Publish the event
        ROS_DEBUG("Publishing a dog position event");
        
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
