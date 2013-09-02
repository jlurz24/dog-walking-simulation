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

      //! Publisher for the future dog position visualization.
      ros::Publisher futureDogVizPub_;
      
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
      
      //! Dog planned position client
      ros::ServiceClient futureDogPositionClient;
      
      //! Last dog position
      geometry_msgs::PoseStamped lastDogPose;
      
      //! Velocity history
      deque<geometry_msgs::TwistStamped> velocityHistory;
      
      //! Known position window (in seconds)
      double knownPositionWindow;
      
   public:
      //! ROS node initialization
      DogPositionDetector():pnh_("~"){
        // Get the future window parameter
        pnh_.param<double>("known_position_window", knownPositionWindow, 0.0);
        
        // TODO: Move these to a scope
        dogVizPub_ = nh_.advertise<visualization_msgs::Marker>("/dog_position_viz", 1);
        dogPositionPub_ = nh_.advertise<DogPosition>("/dog_position", 1);
        futureDogVizPub_ = nh_.advertise<visualization_msgs::Marker>("/future_dog_position_viz", 1);
        
        ros::service::waitForService("/gazebo/get_model_state");
        ros::service::waitForService("/dogsim/dog_planned_position");
        
        modelStateServ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        futureDogPositionClient = nh_.serviceClient<GetDogPlannedPosition>("/dogsim/dog_planned_position", true /* persist */);
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
        
        geometry_msgs::PoseStamped getFutureDogPose(const ros::Time& time){
            GetDogPlannedPosition plannedDogPosition;
            plannedDogPosition.request.time = time;
            futureDogPositionClient.call(plannedDogPosition);
            geometry_msgs::PoseStamped result;
            result.header = plannedDogPosition.response.point.header;
            result.pose.position = plannedDogPosition.response.point.point;
            return result;
        }
        
    void callback(const ros::TimerEvent& event){
        
        // Lookup the current position of the dog.
        geometry_msgs::PoseStamped dogPose = getDogPose(event.current_real);
        
        // Lookup the previous position for velocity calculations.
        geometry_msgs::TwistStamped newDogV;
        newDogV.header = dogPose.header;
        
        double deltaT = dogPose.header.stamp.toSec() - lastDogPose.header.stamp.toSec();
        if(deltaT > numeric_limits<double>::min()){
            newDogV.twist.linear.x = (dogPose.pose.position.x - lastDogPose.pose.position.x) / deltaT;
            newDogV.twist.linear.y = (dogPose.pose.position.y - lastDogPose.pose.position.y) / deltaT;
            newDogV.twist.linear.z = (dogPose.pose.position.z - lastDogPose.pose.position.z) / deltaT;
        }
        lastDogPose = dogPose;
        
        // Direct velocity measurements are too noisy. Use a windowed moving average.
        velocityHistory.push_back(newDogV);
        if(velocityHistory.size() > 10){
            velocityHistory.pop_front();
        }
        
        geometry_msgs::TwistStamped dogV;
        dogV.header = dogPose.header;
        for(unsigned int i = 0; i < velocityHistory.size(); ++i){
            dogV.twist.linear.x += velocityHistory[i].twist.linear.x;
            dogV.twist.linear.y += velocityHistory[i].twist.linear.y;
            dogV.twist.linear.z += velocityHistory[i].twist.linear.z;
        }
        
        dogV.twist.linear.x /= static_cast<double>(velocityHistory.size());
        dogV.twist.linear.y /= static_cast<double>(velocityHistory.size());
        dogV.twist.linear.z /= static_cast<double>(velocityHistory.size());
        
        // Visualize the dog.
        if(dogVizPub_.getNumSubscribers() > 0){
            std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
            dogVizPub_.publish(utils::createMarker(dogPose.pose.position, dogPose.header, BLUE, true));
        }
        DogPosition dogPositionMsg;
        dogPositionMsg.pose = dogPose;
        dogPositionMsg.twist = dogV;

        // Get the future position of the dog.
        if(knownPositionWindow > 0){
            dogPositionMsg.futurePose = getFutureDogPose(ros::Time(event.current_real.toSec() + knownPositionWindow));
            dogPositionMsg.futurePoseKnown = true;
            if(futureDogVizPub_.getNumSubscribers() > 0){
                std_msgs::ColorRGBA GREEN = utils::createColor(0, 1, 0);
                futureDogVizPub_.publish(utils::createMarker(dogPositionMsg.futurePose.pose.position, dogPositionMsg.futurePose.header, GREEN, true));
            }
        }
        else {
            dogPositionMsg.futurePoseKnown = false;
        }
        
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
