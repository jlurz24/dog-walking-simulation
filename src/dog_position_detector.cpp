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
      ros::Publisher dogVizPubPerm;
      
      //! Publisher for the ephemeral dogVizPub;
      ros::Publisher dogVizPubEphem;
      
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
      DogPositionDetector():pnh("~"){
        dogVizPubPerm = nh.advertise<visualization_msgs::Marker>("/dog_position_detector/dog_position_viz_perm", 1);
        dogVizPubEphem = nh.advertise<visualization_msgs::Marker>("/dog_position_detector/dog_position_viz_ephem", 1);
        dogPositionPub = nh.advertise<DogPosition>("/dog_position_detector/dog_position", 1);
        
        ros::service::waitForService("/gazebo/get_model_state");
        
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer = nh.createTimer(ros::Duration(0.1), &DogPositionDetector::callback, this);
        timer.start();
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
        if(dogVizPubPerm.getNumSubscribers() > 0){
            std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
            dogVizPubPerm.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));
        }
        if(dogVizPubEphem.getNumSubscribers() > 0){
            std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
            dogVizPubEphem.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, false));
        }
        DogPosition dogPositionMsg;
        dogPositionMsg.pose = dogPose;
        
        // Publish the event
        ROS_DEBUG("Publishing a dog position event");
        
        dogPositionPub.publish(dogPositionMsg);
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
