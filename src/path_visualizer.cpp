#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/GetPath.h>
#include <dogsim/MaximumTime.h>
#include <dogsim/utils.h>

namespace {
  using namespace std;
  using namespace dogsim;

class PathVisualizer {
private:
  
  //! Node handle
  ros::NodeHandle nh;

  //! Private nh
  ros::NodeHandle pnh;

  //! Publisher for goals with permanent flag
  ros::Publisher goalPubPerm;

  //! Publisher for goals with ephemeral flag
  ros::Publisher goalPubEphem;
  
  //! Publisher for complete goal path.
  ros::Publisher goalPubComplete;
  
  //! Timer that display the goal
  ros::Timer displayTimer;

  //! Timer to broadcast the entire path
  ros::Timer displayTimerComplete;
  
  //! Cached service client.
  ros::ServiceClient getPathClient;
  
  double maximumTime;
public:
  //! ROS node initialization
  PathVisualizer(): pnh("~"){
    
    // Set up the publisher
    goalPubPerm = nh.advertise<visualization_msgs::Marker>("path/walk_goal_viz_perm", 1);
    goalPubEphem = nh.advertise<visualization_msgs::Marker>("path/walk_goal_viz_ephem", 1);
    goalPubComplete = nh.advertise<visualization_msgs::Marker>("path/walk_goal_viz_complete", 1);
   
    ros::service::waitForService("/dogsim/get_path");
    ros::service::waitForService("/dogsim/maximum_time");
    getPathClient = nh.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);
    
    // Precalc the maximum time
    ros::ServiceClient maxTimeClient = nh.serviceClient<dogsim::MaximumTime>("/dogsim/maximum_time", false);
    // Fetch the maximum time.
    dogsim::MaximumTime maxTime;
    if(!maxTimeClient.call(maxTime)){
        ROS_ERROR("Failed to call maximum time");
    }
    maximumTime = maxTime.response.maximumTime;
            
    displayTimer = nh.createTimer(ros::Duration(0.1), &PathVisualizer::displayCallback, this);
    displayTimerComplete = nh.createTimer(ros::Duration(0.25), &PathVisualizer::displayCompleteCallback, this);
  }

  geometry_msgs::PointStamped getDogGoalPosition(const ros::Time& time, bool& started, bool& ended){
      // Determine the goal.
      GetPath getPath;
      getPath.request.time = time.toSec();
      getPathClient.call(getPath);
      started = getPath.response.started;
      ended = getPath.response.ended;
      return getPath.response.point;
  }
  
  void displayFullPath(){
        if(goalPubComplete.getNumSubscribers() > 0){
            
            static const std_msgs::ColorRGBA ORANGE = utils::createColor(0.66, 0.33, 0, 0.5);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.ns = "dogsim";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color = ORANGE;
            marker.scale.x = 0.25;
            marker.scale.y = 0.06;
            marker.scale.z = 0.01;
            
            for(double t = 0; t <= maximumTime; t += 0.5){
                bool started, ended;
                geometry_msgs::PointStamped goal = getDogGoalPosition(ros::Time(t), started, ended);
                assert(goal.header.frame_id != "");
                marker.points.push_back(goal.point);
            }
            goalPubComplete.publish(marker);  
        }
  }

  void displayCompleteCallback(const ros::TimerEvent& event){
      displayFullPath();
  }
  
  void displayCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received display callback");
      if(goalPubPerm.getNumSubscribers() > 0 || goalPubEphem.getNumSubscribers() > 0){
        bool started;
        bool ended;
        const geometry_msgs::PointStamped goal = getDogGoalPosition(event.current_real, started, ended);
        if(!started){
            return;
        }
      
        if(ended){
            ROS_INFO("Walk is ended");
            displayTimer.stop();
            return;
        }

        // Visualize the goal.
        std_msgs::ColorRGBA YELLOW = utils::createColor(0.5, 0.5, 0);
        goalPubPerm.publish(utils::createMarker(goal.point, goal.header, YELLOW, true));
        
        std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
        visualization_msgs::Marker marker;
        marker.header = goal.header;
        marker.ns = "dogsim";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = goal.point;
        marker.color = BLUE;
        marker.color.a = 0.65;
        marker.scale.x = 0.25;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // Goal positions are at height zero, so offset to avoid the marker going
        // below the plane
        marker.pose.position.z += (marker.scale.z / 2.0);
        goalPubEphem.publish(marker);
      }
  }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "path_visualizer");

  PathVisualizer driver;
  ros::spin();
}
