#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

// Generated messages
#include <dogsim/AdjustDogPositionAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

namespace {
  using namespace std;
  using namespace geometry_msgs;

  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

  class HighArmPositionAction {
    public:
      HighArmPositionAction(const string& name): pnh("~"),
                                                 as(nh, name, boost::bind(&HighArmPositionAction::adjust, this, _1), false),
                                                 actionName(name),
                                                 rightArm("right_arm"),
                                                 torsoClient("torso_controller/position_joint_action", true),
                                                 torsoHeight(0){
        
        as.registerPreemptCallback(boost::bind(&HighArmPositionAction::preemptCB, this));
        pnh.param<double>("torso_height", torsoHeight, 0.1);
        torsoClient.waitForServer();
        as.start();
    }
  
    private:
  
        void preemptCB(){
            ROS_DEBUG("Preempting the high arm position action");

            if(!as.isActive()){
                ROS_DEBUG("High arm position action canceled prior to start");
                return;
            }

            rightArm.stop();
            torsoClient.cancelGoal();
            as.setPreempted();
        }
  
  bool adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    ROS_DEBUG("Adjusting dog position");
      
    if(!as.isActive()){
        ROS_INFO("Adjust dog position action cancelled prior to start");
        return false;
    }
    
    ROS_INFO("Adjusting torso to height %f", torsoHeight);
    pr2_controllers_msgs::SingleJointPositionGoal up;
    up.position = torsoHeight;
    up.min_duration = ros::Duration(0.0);
    up.max_velocity = 2.0;
    
    torsoClient.sendGoal(up);
    torsoClient.waitForResult();
   
    ROS_INFO("Moving arm to high position");
    vector<double> positions(7);
    positions[0] = -1.55;
    positions[1] = -0.5;
    positions[2] = 0.0;
    positions[3] = -1.2;
    positions[4] = 0.0;
    positions[5] = 0.0;
    positions[6] = 0.0;
    rightArm.setJointValueTarget(positions);
    rightArm.move(); 
    as.setSucceeded();
    return true;
  }
  
    protected:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        
        // Actionlib classes
        actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
        string actionName;
        move_group_interface::MoveGroup rightArm;
        TorsoClient torsoClient;
        double torsoHeight;
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "high_arm_position_action");
  HighArmPositionAction action(ros::this_node::getName());
  ros::spin();
  
  return 0;
}
