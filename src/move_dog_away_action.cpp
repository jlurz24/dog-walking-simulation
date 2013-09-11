#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/server/simple_action_server.h>

// Generated messages
#include <dogsim/MoveDogAwayAction.h>

namespace {
    using namespace std;
    using namespace ros;

    class MoveDogAway {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::MoveDogAwayAction> as;
            string actionName;
            move_group_interface::MoveGroup rightArm;

        public:
            MoveDogAway(const string& name):as(nh, name, boost::bind(&MoveDogAway::moveAway, this), false),
                                    actionName(name),
                                    rightArm("right_arm"){
            as.registerPreemptCallback(boost::bind(&MoveDogAway::preemptCB, this));
            as.start();
        }

            void preemptCB(){
                ROS_DEBUG("Preempting the move dog away action");

                if(!as.isActive()){
                    ROS_DEBUG("Adjust dog position action cancelled prior to start");
                    return;
                }

                rightArm.stop();
                as.setPreempted();
        }
        
        void moveAway(){
            if(!as.isActive()){
                ROS_INFO("Move dog away action cancelled prior to start");
                return;
            }
            
            ROS_INFO("Moving dog out of the way");
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
  ros::init(argc, argv, "move_dog_away");
  MoveDogAway action(ros::this_node::getName());
  ros::spin();
}
