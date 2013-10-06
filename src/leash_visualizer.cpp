#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <dogsim/utils.h>

namespace {
  using namespace std;

class LeashVisualizer {
private:
  
  //! Node handle
  ros::NodeHandle nh;

  //! Publisher for leash
  ros::Publisher leashPub;
  
  //! Timer that display the goal
  ros::Timer displayTimer;
  
  //! Cached service client.
  ros::ServiceClient modelStateServ;
  
  //! Transform listener
  tf::TransformListener tf;
  
public:
  //! ROS node initialization
  LeashVisualizer(){
    
    // Set up the publisher
    leashPub = nh.advertise<visualization_msgs::Marker>("leash_visualizer/leash_viz", 1);
    ros::service::waitForService("/gazebo/get_model_state");
    modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
    displayTimer = nh.createTimer(ros::Duration(0.025), &LeashVisualizer::displayCallback, this);
  }
  
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

  void displayCallback(const ros::TimerEvent& event){
      if(leashPub.getNumSubscribers() > 0){
        // First fetch the dog position
        // Visualize the goal.
        geometry_msgs::PoseStamped dogPose = getDogPose(event.current_real);
        
        // Convert to the hand frame
        geometry_msgs::PoseStamped dogInHandFrame;
        try {
            tf.transformPose("/r_wrist_roll_link", ros::Time(0), dogPose, dogPose.header.frame_id, dogInHandFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform dog position to /r_wrist_roll_link");
            return;
        }
        
        std_msgs::ColorRGBA GREEN = utils::createColor(0.0, 1.0, 0.0);
        visualization_msgs::Marker points;
        points.header = dogInHandFrame.header;
        points.ns = "dogsim";
        points.id = 0;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.x = points.pose.orientation.y = points.pose.orientation.z = 0.0;
        points.pose.orientation.w = 1.0;
        
        points.color = GREEN;
        points.scale.x = 0.04;
        
        geometry_msgs::Point base;
        points.points.push_back(base);
        
        geometry_msgs::Point dog;
        dog.x = dogInHandFrame.pose.position.x;
        dog.y = dogInHandFrame.pose.position.y;
        dog.z = dogInHandFrame.pose.position.z;
        points.points.push_back(dog);
        
        leashPub.publish(points);
      }
  }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "leash_visualizer");

  LeashVisualizer driver;
  ros::spin();
}
