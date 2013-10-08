#include <ros/ros.h>
#include <dogsim/utils.h>
#include <gazebo_msgs/GetModelState.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetDogOrientation.h>

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

      //! Publisher for the dog positon
      ros::Publisher dogDirectionVizPub;
      
      //! Frequency at which to search for the dog
      ros::Timer timer;
      
      //! The node handle we'll be using
      ros::NodeHandle nh;

      //! Private nh
      ros::NodeHandle pnh;

      //! Cached service client.
      ros::ServiceClient modelStateServ;
      ros::ServiceClient dogOrientationServ;
   public:
      //! ROS node initialization
      DogPositionDetector():pnh("~"){
        dogVizPubPerm = nh.advertise<visualization_msgs::Marker>("/dog_position_detector/dog_position_viz_perm", 1);
        dogVizPubEphem = nh.advertise<visualization_msgs::Marker>("/dog_position_detector/dog_position_viz_ephem", 1);
        dogDirectionVizPub = nh.advertise<visualization_msgs::Marker>("/dog_position_detector/dog_direction_viz", 1);
        dogPositionPub = nh.advertise<DogPosition>("/dog_position_detector/dog_position", 1);
        
        ros::service::waitForService("/gazebo/get_model_state");
        ros::service::waitForService("/dogsim/dog_orientation");
        
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        dogOrientationServ = nh.serviceClient<dogsim::GetDogOrientation>("/dogsim/dog_orientation", true /* persistent */);
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
        
        geometry_msgs::QuaternionStamped getDogOrientation(){
            dogsim::GetDogOrientation dogOrientation;
            if(!dogOrientationServ.call(dogOrientation)){
                ROS_ERROR("Failed to get dog orientation");
            }
            return dogOrientation.response.orientation;
        }
        
    void callback(const ros::TimerEvent& event){
        
        // Lookup the current position of the dog.
        geometry_msgs::PoseStamped dogPose = getDogPose(event.current_real);
        
        // Visualize the dog.
        static const std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
        if(dogVizPubPerm.getNumSubscribers() > 0){
            dogVizPubPerm.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));
        }
        
        if(dogVizPubEphem.getNumSubscribers() > 0){
            visualization_msgs::Marker marker;
            marker.header = dogPose.header;
            marker.ns = "dogsim";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = dogPose.pose;
            marker.color = BLUE;
            marker.scale.x = 0.25;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            
            dogVizPubEphem.publish(marker);
        }
        
        if(dogDirectionVizPub.getNumSubscribers() > 0){
            // Get the direction
            static const double PI = boost::math::constants::pi<double>();

            geometry_msgs::QuaternionStamped orientation = getDogOrientation();
            visualization_msgs::Marker marker;
            marker.header = orientation.header;
            marker.ns = "dogsim";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = dogPose.pose.position;
            marker.pose.orientation = orientation.quaternion;
            marker.color = BLUE;
            marker.color.a = 1;
            marker.scale.x = 0.4;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            dogDirectionVizPub.publish(marker);
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
