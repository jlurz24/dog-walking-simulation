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

      //! Cached service client.
      ros::ServiceClient modelStateServ;
      
      //! Last dog position
      geometry_msgs::PoseStamped lastDogPose;
      
   public:
      //! ROS node initialization
      DogPositionDetector():pnh_("~"){
        dogVizPub_ = nh_.advertise<visualization_msgs::Marker>("/dog_position_viz", 1);
        dogPositionPub_ = nh_.advertise<dogsim::DogPosition>("/dog_position", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        lastDogPose = getDogPose(ros::Time::now());
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
        
        // Lookup the previous position for velocity calculations.
        geometry_msgs::TwistStamped dogV;
        dogV.header = dogPose.header;
        
        double deltaT = dogPose.header.stamp.toSec() - lastDogPose.header.stamp.toSec();
        if(deltaT > numeric_limits<double>::min()){
            dogV.twist.linear.x = (dogPose.pose.position.x - lastDogPose.pose.position.x) / deltaT;
            dogV.twist.linear.y = (dogPose.pose.position.y - lastDogPose.pose.position.y) / deltaT;
            dogV.twist.linear.z = (dogPose.pose.position.z - lastDogPose.pose.position.z) / deltaT;
        }
        lastDogPose = dogPose;
        
        // Visualize the dog.
        std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
        dogVizPub_.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));

        // Determine the relative dog position
        // TODO: Should we instead publish in the map frame?
        dogsim::DogPosition dogPositionMsg;
        dogPositionMsg.header.frame_id = "/base_footprint";
        dogPositionMsg.header.stamp = event.current_real;
        try {
            tf_.transformPose("/base_footprint", ros::Time(0), dogPose, dogPose.header.frame_id, dogPositionMsg.pose);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform dog point to /base_footprint");
            return;
        }
        try {
            geometry_msgs::Vector3Stamped twistVector;
            twistVector.header = dogV.header;
            twistVector.vector.x = dogV.twist.linear.x;
            twistVector.vector.y = dogV.twist.linear.y;
            twistVector.vector.z = dogV.twist.linear.z;
            geometry_msgs::Vector3Stamped twistVectorInBaseFrame;
            tf_.transformVector("/base_footprint", ros::Time(0), twistVector, twistVector.header.frame_id, twistVectorInBaseFrame);
            dogPositionMsg.twist.twist.linear.x = twistVectorInBaseFrame.vector.x;
            dogPositionMsg.twist.twist.linear.y = twistVectorInBaseFrame.vector.y;
            dogPositionMsg.twist.twist.linear.z = twistVectorInBaseFrame.vector.z;
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform dog twist to /base_footprint");
            return;
        }
        dogPositionMsg.pose.header = dogPositionMsg.header;
        dogPositionMsg.twist.header = dogPositionMsg.header;

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
