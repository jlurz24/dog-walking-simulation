#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>

// Generated messages
#include <dogsim/MoveArmToClearPositionAction.h>

namespace {
    using namespace std;
    using namespace ros;

    class MoveArmToClearPosition {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::MoveArmToClearPositionAction> as;
            string actionName;
            move_group_interface::MoveGroup rightArm;

        public:
            MoveArmToClearPosition(const string& name):as(nh, name, boost::bind(&MoveArmToClearPosition::moveToClearPosition, this), false),
                                    actionName(name),
                                    rightArm("right_arm"){
            as.registerPreemptCallback(boost::bind(&MoveArmToClearPosition::preemptCB, this));
            as.start();
        }

            void preemptCB(){
                ROS_DEBUG("Preempting the move arm to clear position action");

                if(!as.isActive()){
                    ROS_DEBUG("Move arm to clear position action cancelled prior to start");
                    return;
                }

                rightArm.stop();
                as.setPreempted();
        }
        
        void moveToClearPosition(){
            if(!as.isActive()){
                ROS_INFO("Move arm to clear position cancelled prior to start");
                return;
            }
            
            ROS_INFO("Moving arm to clear position");
            vector<double> positions(7);
            positions[0] = -2.0;
            positions[3] = -1.2;
            positions[5] = 0.0;
            rightArm.setJointValueTarget(positions);
            rightArm.move();
            as.setSucceeded();
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_arm_to_clear_position");
  MoveArmToClearPosition action(ros::this_node::getName());
  ros::spin();
}
