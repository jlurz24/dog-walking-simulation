#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/GetPath.h>
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
  
  //! Timer that display the goal
  ros::Timer displayTimer;
  
  //! Cached service client.
  ros::ServiceClient getPathClient;
  
public:
  //! ROS node initialization
  PathVisualizer(): pnh("~"){
    
    // Set up the publisher
    goalPubPerm = nh.advertise<visualization_msgs::Marker>("path/walk_goal_viz_perm", 1);
    goalPubEphem = nh.advertise<visualization_msgs::Marker>("path/walk_goal_viz_ephem", 1);
        
    ros::service::waitForService("/dogsim/get_path");
    getPathClient = nh.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);
    displayTimer = nh.createTimer(ros::Duration(0.2), &PathVisualizer::displayCallback, this);
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

  void displayCallback(const ros::TimerEvent& event){
      ROS_DEBUG("Received display callback");
      if(goalPubPerm.getNumSubscribers() > 0 || goalPubEphem.getNumSubscribers() > 0){
        bool started;
        bool ended;
        const geometry_msgs::PointStamped goal = getDogGoalPosition(event.current_real, started, ended);
        assert(started);
      
        if(ended){
            ROS_INFO("Walk is ended");
            displayTimer.stop();
            return;
        }

        // Visualize the goal.
        std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
        goalPubPerm.publish(utils::createMarker(goal.point, goal.header, RED, true));
        goalPubEphem.publish(utils::createMarker(goal.point, goal.header, RED, false));
      }
  }
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "path_visualizer");

  PathVisualizer driver;
  ros::spin();
}
