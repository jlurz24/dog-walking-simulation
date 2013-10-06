#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Generated messages
#include <dogsim/AdjustDogPositionAction.h>

namespace {
  using namespace std;
  using namespace geometry_msgs;

  class NoOpAdjustArmPositionAction {
    public:
      NoOpAdjustArmPositionAction(const string& name): as(nh, name, boost::bind(&NoOpAdjustArmPositionAction::adjust, this, _1), false),
                                                 actionName(name){
        
        as.registerPreemptCallback(boost::bind(&NoOpAdjustArmPositionAction::preemptCB, this));
        as.start();
    }
  
    private:
  
        void preemptCB(){
            ROS_DEBUG("Preempting the no-op adjust arm position action");

            if(!as.isActive()){
                ROS_DEBUG("No op arm adjust arm position action cancelled prior to start");
                return;
            }

            as.setPreempted();
        }
  
  bool adjust(const dogsim::AdjustDogPositionGoalConstPtr& goal){
    ROS_DEBUG("Adjusting dog position");
    ros::Duration d(1.0);
    d.sleep();
    as.setSucceeded();
    return true;
  }
  
    protected:
        ros::NodeHandle nh;
    
        // Actionlib classes
        actionlib::SimpleActionServer<dogsim::AdjustDogPositionAction> as;
        string actionName;
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "no_op_adjust_arm_position_action");
  NoOpAdjustArmPositionAction action(ros::this_node::getName());
  ros::spin();
  
  return 0;
}
