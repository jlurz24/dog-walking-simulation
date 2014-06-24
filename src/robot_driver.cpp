#include <ros/ros.h>
#include <dogsim/utils.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/DogPosition.h>
#include <dogsim/GetEntireRobotPath.h>
#include <dogsim/AvoidingDog.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
#include <message_filters/subscriber.h>
#include <dogsim/AdjustDogPositionAction.h>
#include <dogsim/MoveArmToBasePositionAction.h>
#include <dogsim/MoveRobotAction.h>
#include <dogsim/MoveDogAwayAction.h>
#include <actionlib/client/simple_action_client.h>

namespace {
using namespace std;
using namespace dogsim;

typedef actionlib::SimpleActionClient<AdjustDogPositionAction> AdjustDogClient;
typedef actionlib::SimpleActionClient<MoveRobotAction> MoveRobotClient;
typedef actionlib::SimpleActionClient<MoveArmToBasePositionAction> MoveArmToBasePositionClient;

//! Amount of time before starting walk. This provides time for the robot to finish
//  tucking its arms and move to the starting position.
const double DELAY_TIME_DEFAULT = 10.0;

//! Interval to update the move_robot_action
const double MOVE_ROBOT_UPDATE_INTERVAL_DEFAULT = 2.0;

class RobotDriver {
private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Timer that controls completion of the walk
    ros::Timer completeTimer;

    //! One shot timer that performs delayed start
    ros::Timer initTimer;

    //! Dog position subscriber
    auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

    //! Avoiding dog listener.
    auto_ptr<message_filters::Subscriber<AvoidingDog> > avoidingDogSub;

    //! Client for the arm to attempt to position the dog
    AdjustDogClient adjustDogClient;

    //! Client for the movement of the robot base.
    MoveRobotClient moveRobotClient;

    //! Client to adjust the arm of the robot to the start position
    MoveArmToBasePositionClient moveArmToBasePositionClient;

    //! Cached service client.
    ros::ServiceClient getPathClient;
    ros::ServiceClient getEntireRobotPathClient;

    //! Whether the robot is operating on its own
    bool soloMode;

    //! Whether the robot is running in a mode where the base does not move.
    bool noSteeringMode;

    //! Whether are currently avoiding the dog
    bool avoidingDog;

    //! Publishers for starting and stopping measurement.
    ros::Publisher startMeasuringPub;
    ros::Publisher stopMeasuringPub;

    void publishStopMeasurement() {
        // Notify clients to stop measuring.
        position_tracker::StopMeasurement stopMeasuringMsg;
        stopMeasuringMsg.header.stamp = ros::Time::now();
        stopMeasuringPub.publish(stopMeasuringMsg);
    }

public:
    //! ROS node initialization
    RobotDriver() :
            pnh("~"), adjustDogClient("adjust_dog_position_action", true), moveRobotClient(
                    "move_robot_action", true), moveArmToBasePositionClient(
                    "move_arm_to_base_position_action", true),
                    soloMode(false), noSteeringMode(
                    false), avoidingDog(false) {

        ROS_INFO("Initializing the robot driver @ %f", ros::Time::now().toSec());

        ros::service::waitForService("/dogsim/get_path");
        ros::service::waitForService("/dogsim/start");
        ros::service::waitForService("/dogsim/maximum_time");
        ros::service::waitForService("/dogsim/get_entire_robot_path");

        getPathClient = nh.serviceClient<GetPath>("/dogsim/get_path", true /* persist */);

        getEntireRobotPathClient = nh.serviceClient<GetEntireRobotPath>("/dogsim/get_entire_robot_path", true /* persist */);

        moveRobotClient.waitForServer();
        moveArmToBasePositionClient.waitForServer();

        startMeasuringPub = nh.advertise<position_tracker::StartMeasurement>("start_measuring", 1,
                true);
        stopMeasuringPub = nh.advertise<position_tracker::StopMeasurement>("stop_measuring", 1,
                true);

        double delayTimeD;
        pnh.param<double>("start_delay", delayTimeD, DELAY_TIME_DEFAULT);

        ros::Duration delayTime;
        delayTime.fromSec(delayTimeD);

        // Only use the steering callback when in solo mode. Otherwise we'll move based on the required positions to
        // move the arm.
        pnh.param<bool>("solo_mode", soloMode, false);
        if (soloMode) {
            ROS_INFO("Running solo mode");
        }
        else {
            ROS_INFO("Running regular mode");
            adjustDogClient.waitForServer();
        }

        pnh.param<bool>("no_steering_mode", noSteeringMode, false);

        initTimer = nh.createTimer(delayTime, &RobotDriver::init, this,
                true /* One shot */);

        ROS_INFO("Robot driver initialization complete @ %f. Waiting for %f(s) to start.", ros::Time::now().toSec(), delayTime.toSec());
    }

    void init(const ros::TimerEvent& event) {
        ROS_INFO("Entering delayed init");

        MoveArmToBasePositionGoal moveArmToBasePositionGoal;
        utils::sendGoal(&moveArmToBasePositionClient, moveArmToBasePositionGoal, nh, 1.0 /* 10.0 */);

        dogPositionSub.reset(
                new message_filters::Subscriber<DogPosition>(nh,
                        "/dog_position_detector/dog_position", 1));
        dogPositionSub->registerCallback(boost::bind(&RobotDriver::dogPositionCallback, this, _1));

        avoidingDogSub.reset(
                new message_filters::Subscriber<AvoidingDog>(nh, "/avoid_dog/avoiding", 1));
        avoidingDogSub->registerCallback(boost::bind(&RobotDriver::avoidingDogCallback, this, _1));

        startPath(ros::Time::now());


        ROS_INFO("Delayed init complete");
    }

    void startPath(const ros::Time& currentTime) {
        StartPath startPath;
        startPath.request.time = currentTime;
        ros::ServiceClient startPathClient = nh.serviceClient<StartPath>("/dogsim/start", false);
        startPathClient.call(startPath);

        // Calculate the maximum runtime
        dogsim::MaximumTime maxTime;
        ros::ServiceClient maxTimeClient = nh.serviceClient<dogsim::MaximumTime>(
                "/dogsim/maximum_time", false);
        if (!maxTimeClient.call(maxTime)) {
            ROS_ERROR("Failed to call maximum time");
        }

        // Notify clients to start measuring.
        position_tracker::StartMeasurement startMeasuringMsg;
        startMeasuringMsg.header.stamp = ros::Time::now();
        startMeasuringPub.publish(startMeasuringMsg);

        completeTimer = nh.createTimer(maxTime.response.maximumTime, &RobotDriver::walkComplete,
                this, true /* One shot */);

        // Begin moving towards the target
        sendMoveBaseGoal();
    }

    void sendMoveBaseGoal(){
        ROS_INFO("Sending move base goal");
        MoveRobotGoal goal;
        GetEntireRobotPath getPath;
        getPath.request.increment = 0.25;
        getEntireRobotPathClient.call(getPath);
        ROS_INFO("Path has %lu poses", getPath.response.poses.size());
        goal.poses = getPath.response.poses;

        moveRobotClient.sendGoal(goal);
    }

    void avoidingDogCallback(const AvoidingDogConstPtr& avoidDogMsg) {
        ROS_DEBUG("Robot driver received avoid dog callback");
        if (avoidDogMsg->avoiding) {
            ROS_INFO("Canceling movement as the avoiding flag is set");
            // Cancel all base movement. Arm movement will be cancelled
            // by sending the command from the avoid dog client.
            moveRobotClient.cancelGoal();
        }
        else if (avoidingDog) {
            ROS_INFO("Resetting arm as the avoiding flag is not set");
            // Reset the arm
            MoveArmToBasePositionGoal moveArmToBasePositionGoal;
            moveArmToBasePositionClient.sendGoal(moveArmToBasePositionGoal);
            sendMoveBaseGoal();
        }
        // Set the flag
        avoidingDog = avoidDogMsg->avoiding;
    }

    void dogPositionCallback(const DogPositionConstPtr& dogPosition) {
        ROS_DEBUG("Received a dog position callback @ %f", ros::Time::now().toSec());

        // No actions can be executed while we are avoiding the dog.
        if (avoidingDog) {
            return;
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

    void walkComplete(const ros::TimerEvent& event){
        dogPositionSub->unsubscribe();
        avoidingDogSub->unsubscribe();
        // Notify clients to stop measuring.
        publishStopMeasurement();
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_driver");

    RobotDriver driver;
    ros::spin();
    ROS_INFO("Exiting robot driver");
}
