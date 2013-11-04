#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <dogsim/utils.h>
#include <tf/transform_listener.h>

// Generated messages
#include <dogsim/LookAtPathAction.h>

namespace {
using namespace std;
using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class LookAtPathAction {
public:
    LookAtPathAction(const string& name) :
        as(nh, name, boost::bind(&LookAtPathAction::look, this, _1),
                false), actionName(name), pointHeadClient(
                        "/head_traj_controller/point_head_action", true) {

        as.registerPreemptCallback(
                boost::bind(&LookAtPathAction::preemptCB, this));
        pointHeadClient.waitForServer();
        as.start();
    }

private:

    void preemptCB() {
        ROS_INFO("Preempting the look at path action");

        if (!as.isActive()) {
            ROS_DEBUG("Look at path canceled prior to start");
            return;
        }
        pointHeadClient.cancelAllGoals();
        as.setPreempted();
    }

    bool look(const dogsim::LookAtPathGoalConstPtr& goal) {
        ROS_INFO("Initiating look at path");

        pr2_controllers_msgs::PointHeadGoal phGoal;
        phGoal.target = goal->futurePathPosition;
        // rosrun tf tf_echo base_link wide_stereo_link
        phGoal.target.point.z = 1.25;
        pointHeadClient.sendGoal(phGoal);
        if(!pointHeadClient.waitForResult(ros::Duration(1.0))){
            ROS_INFO("Look at path timed out");
            pointHeadClient.cancelGoal();
            as.setAborted();
            return false;
        }
        if(pointHeadClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Look at path succeeded");
            as.setSucceeded();
            return true;
        }
        ROS_INFO("Look at path failed: %s", pointHeadClient.getState().toString().c_str());
        as.setAborted();
        return false;
    }

protected:
    ros::NodeHandle nh;

    // Actionlib classes
    actionlib::SimpleActionServer<dogsim::LookAtPathAction> as;
    string actionName;
    PointHeadClient pointHeadClient;
    tf::TransformListener tf;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "look_at_path_action");
    LookAtPathAction action(ros::this_node::getName());
    ros::spin();

    return 0;
}
