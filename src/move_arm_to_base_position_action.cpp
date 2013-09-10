#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/math/constants/constants.hpp>

// Generated messages
#include <dogsim/MoveArmToBasePositionAction.h>

namespace {
    using namespace std;
    using namespace ros;

    class MoveArmToBasePosition {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::MoveArmToBasePositionAction> as;
            string actionName;
            move_group_interface::MoveGroup rightArm;

        public:
            MoveArmToBasePosition(const string& name):as(nh, name, boost::bind(&MoveArmToBasePosition::moveToBasePosition, this), false),
                                    actionName(name),
                                    rightArm("right_arm"){
            as.start();
        }

        void moveToBasePosition(){
            if(!as.isActive()){
                ROS_INFO("Move arm to base position action cancelled prior to start");
                return;
            }
            
            ROS_INFO("Moving arm to base position");
            vector<double> positions(7);
            positions[0] = -boost::math::constants::pi<double>() / 4.0;
            positions[1] = 0.4;
            positions[2] = -boost::math::constants::pi<double>() / 4.0;
            positions[3] = -boost::math::constants::pi<double>() / 4.0;
            positions[4] = -1.0;
            positions[5] = -0.2;
            positions[6] = 0.0;
            rightArm.setJointValueTarget(positions);
            rightArm.move();
            
            as.setSucceeded();
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_arm_to_base_position_action");
  MoveArmToBasePosition action(ros::this_node::getName());

  ros::spin();
}
