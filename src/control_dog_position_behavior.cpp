#include <ros/ros.h>

// Generated messages
#include <dogsim/ControlDogPositionAction.h>
#include <dogsim/AdjustDogPositionAction.h>
#include <dogsim/DogPosition.h>
#include <actionlib/client/simple_action_client.h>
#include <message_filters/subscriber.h>
#include <dogsim/GetPath.h>
#include <actionlib/server/simple_action_server.h>

namespace {
    using namespace std;
    using namespace ros;
    using namespace dogsim;

    typedef actionlib::SimpleActionClient<AdjustDogPositionAction> AdjustDogClient;

    class ControlDogPositionBehavior {
        private:
            NodeHandle nh;
            actionlib::SimpleActionServer<dogsim::ControlDogPositionAction> as;
            string actionName;

            //! Client for the arm to attempt to position the dog
            AdjustDogClient adjustDogClient;

            //! Dog position subscriber
            auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

            //! Cached service client.
            ros::ServiceClient getPathClient;

            bool active;
        public:
            ControlDogPositionBehavior(const string& name):as(nh, name, boost::bind(&ControlDogPositionBehavior::activate, this), false),
                                    actionName(name),
                                    adjustDogClient("adjust_dog_position_action", true){
            as.registerPreemptCallback(boost::bind(&ControlDogPositionBehavior::deactivate, this));
            dogPositionSub.reset(
                    new message_filters::Subscriber<DogPosition>(nh,
                            "/dog_position_detector/dog_position", 1));
            dogPositionSub->registerCallback(boost::bind(&ControlDogPositionBehavior::dogPositionCallback, this, _1));
            dogPositionSub->unsubscribe();
            adjustDogClient.waitForServer();
            getPathClient = nh.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);
            ros::service::waitForService("/dogsim/get_path");
            active = false;
            as.start();
        }

        void deactivate(){
            ROS_DEBUG("Deactivating the ControlDogPositionBehavior");

            if(!as.isActive()){
                ROS_DEBUG("Deactivating the ControlDogPositionBehavior prior to start");
                return;
            }
            active = false;
            dogPositionSub->unsubscribe();
            adjustDogClient.cancelGoal();
            as.setPreempted();
        }
        
        void activate(){
            ROS_DEBUG("Activating the ControlDogPositionBehavior");
            if(!as.isActive()){
                ROS_INFO("ControlDogPositionBehavior cancelled prior to start");
                return;
            }
            active = true;
            dogPositionSub->subscribe();
            as.setSucceeded();
        }

        void dogPositionCallback(const DogPositionConstPtr& dogPosition) {

            ROS_DEBUG("Received a dog position callback @ %f", ros::Time::now().toSec());

            if(!active){
                "Received a dog position message while inactive";
            }

            if (dogPosition->unknown || dogPosition->stale) {
                ROS_DEBUG("Dog position is unknown or stale. Canceling movement of arm.");
                adjustDogClient.cancelGoal();
                return;
            }

            bool ended = false;
            bool started = false;
            const geometry_msgs::PointStamped goalCurrent = getDogGoalPosition(
                    ros::Time(ros::Time::now().toSec()), started, ended);

            // Check for completion
            if (!started || ended) {
                return;
            }

            // Only adjust dog position if the last adjustment finished
            if (adjustDogClient.getState() != actionlib::SimpleClientGoalState::ACTIVE) {
                ROS_DEBUG("Sending new adjust dog goal");
                AdjustDogPositionGoal adjustGoal;
                adjustGoal.dogPose = dogPosition->pose;
                adjustGoal.goalPosition = goalCurrent;
                adjustDogClient.sendGoal(adjustGoal);
            }
            ROS_DEBUG("Completed dog position callback");
        }

        geometry_msgs::PointStamped getDogGoalPosition(const ros::Time& time, bool& started,
                bool& ended) {
            // Determine the goal.
            GetPath getPath;
            getPath.request.time = time;
            getPathClient.call(getPath);
            started = getPath.response.started;
            ended = getPath.response.ended;
            return getPath.response.point;
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "control_dog_position_behavior");
  ControlDogPositionBehavior action(ros::this_node::getName());
  ros::spin();
}
