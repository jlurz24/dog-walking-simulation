#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <physics/Model.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/math/constants/constants.hpp>
#include <boost/random/uniform_real.hpp>
#include <stdlib.h>
#include <dogsim/utils.h>
#include <ros/ros.h>
#include <dogsim/GetPath.h>
#include <dogsim/StartPath.h>
#include <dogsim/MaximumTime.h>
#include <tf/tf.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>


namespace {

using namespace std;
using namespace gazebo;

const double pi = boost::math::constants::pi<double>();

const double KD_DEFAULT = 0.15;

// Probability of starting a new gaussian in each second
const double P_NEW_GAUSS_DEFAULT = 0.16;

// Multiple of sigma that captures nearly half of a gaussians width.
const double GAUSS_HALF_WIDTH = 3;

const double KP_DEFAULT = 0.0075;

const double MAXIMUM_FORCE = 13.5;

// Number of simulator iterations per second.
const unsigned int SIMULATOR_CYCLES_PER_SECOND = 100;

// Rate to run the updates at
const double UPDATE_RATE = 0.001;

const double MIN_DISTANCE_FROM_ROBOT = 0.5;
const double BASE_RADIUS = sqrt(2 * utils::square(0.668 / 2.0));
const double AVOIDANCE_FORCE_MULT = 1.25;

class DogModelPlugin: public ModelPlugin {
public:
    DogModelPlugin() {
        ROS_INFO("Creating Dog Plugin");
    }

    ~DogModelPlugin() {
        ROS_INFO("Destroying Dog Plugin");
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        ROS_INFO("Loading Dog Plugin");

        // Store the pointer to the model
        this->model = _parent;

        // Save the start time.
        this->previousTime = this->model->GetWorld()->GetSimTime();

        this->previousErrorX = this->previousErrorY = 0.0;
        this->forceX = this->forceY = this->forceZ = 0.0;
        this->appliedForceX = this->appliedForceY = 0.0;

        getPathClient = nh.serviceClient<dogsim::GetPath>("/dogsim/get_path", true);

        // Fetch the body link.
        body = this->model->GetLink("body");

        waitForService("/dogsim/get_path");
        waitForService("/dogsim/start");
        waitForService("/dogsim/maximum_time");

        dogGoalVizPub = nh.advertise<visualization_msgs::Marker>("dogsim/dog_goal_viz", 1);

        // Initialize the gaussians.
        bool addGaussians;
        nh.param<bool>("add_gaussians_to_path", addGaussians, true);
        if (addGaussians) {
            initGaussians();
        }

        startMeasuringPub = nh.advertise<position_tracker::StartMeasurement>("start_measuring", 1,
                true);
        stopMeasuringPub = nh.advertise<position_tracker::StopMeasurement>("stop_measuring", 1,
                true);

        // Start the path if we are in solo mode. In regular mode the robot does this.
        bool isSoloDog;
        nh.param<bool>("solo_dog", isSoloDog, false);
        if (isSoloDog) {
            // Notify clients to start measuring.
            position_tracker::StartMeasurement startMeasuringMsg;
            startMeasuringMsg.header.stamp = ros::Time::now();
            startMeasuringPub.publish(startMeasuringMsg);
            startPath();
        }

        nh.param<double>("dog_kp", KP, KP_DEFAULT);
        nh.param<double>("dog_kd", KD, KD_DEFAULT);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&DogModelPlugin::OnUpdate, this));
    }

private:
    /**
     * This is not a standard ROS thread, so waitForService does not work. Mimic it with a simple spin-wait.
     */
    void waitForService(const string& serviceName) {
        while (!ros::service::exists(serviceName, false)) {
            // Spin-wait
            ROS_INFO("Waiting for service %s", serviceName.c_str());
        }
    }

    void startPath() {
        dogsim::StartPath startPath;
        startPath.request.time = ros::Time(this->model->GetWorld()->GetSimTime().Double());
        ros::ServiceClient startPathClient = nh.serviceClient<dogsim::StartPath>("/dogsim/start",
                false);
        if (!startPathClient.call(startPath)) {
            ROS_ERROR("Failed to start path");
        }
    }

    void initGaussians() {
        dogsim::MaximumTime maxTime;
        ros::ServiceClient maxTimeClient = nh.serviceClient<dogsim::MaximumTime>(
                "/dogsim/maximum_time", false);
        if (!maxTimeClient.call(maxTime)) {
            ROS_ERROR("Failed to call maximum time");
        }

        // Determine if we should start a new gaussian.
        boost::uniform_real<> randomZeroToOne(0, 1);

        double pNewGauss;
        nh.param<double>("p_new_gauss", pNewGauss, P_NEW_GAUSS_DEFAULT);
        double gaussHeight;
        nh.param<double>("gauss_height", gaussHeight, 1.0);
        double gaussMinWidth;
        nh.param<double>("gauss_min_width", gaussMinWidth, 0.5);
        double gaussMaxWidth;
        nh.param<double>("gauss_max_width", gaussMaxWidth, 8.0);
        ROS_INFO(
                "Gaussian parameters -  pNewGauss: %f gaussHeight: %f gaussMinWidth: %f gaussMaxWidth: %f",
                pNewGauss, gaussHeight, gaussMinWidth, gaussMaxWidth);

        ROS_INFO("Initializing gaussians. Maximum time is %f",
                maxTime.response.maximumTime.toSec());
        // Precompute all the lissajous cycles.
        for (double t = 0; t <= maxTime.response.maximumTime.toSec();
                t += 1.0 / SIMULATOR_CYCLES_PER_SECOND) {
            if (randomZeroToOne(rng)
                    < pNewGauss / static_cast<double>(SIMULATOR_CYCLES_PER_SECOND)) {
                // Create a structure with the parameters.
                GaussParams params;
                boost::uniform_real<> randomA(-gaussHeight * pi, gaussHeight * pi);
                params.a = randomA(rng);

                boost::uniform_real<> randomC(gaussMinWidth * pi, gaussMaxWidth * pi);
                params.c = randomC(rng);

                ROS_DEBUG("New gaussian created with a: %f c: %f at time %f", params.a, params.c,
                        t);
                params.startTime = t;
                gaussParams.push_back(params);
            }
        }
        ROS_INFO("Completed initializing gaussians. Total gaussians is %lu", gaussParams.size());
    }

    // Called by the world update start event
    void OnUpdate() {

        // Calculate the desired position.
        common::Time currTime = this->model->GetWorld()->GetSimTime();
        if (currTime - this->previousTime >= common::Time::SecToNano(UPDATE_RATE)) {
            bool running, ended;
            math::Vector3 goalPosition = calcGoalPosition(currTime, running, ended);
            if(ended){
                ROS_INFO("Stopping dog model movement");
                // Stop updates.
                event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
                bool isSoloDog;
                nh.param<bool>("solo_dog", isSoloDog, false);
                if (isSoloDog) {
                    // Notify clients to stop measuring.
                    position_tracker::StopMeasurement stopMeasuringMsg;
                    stopMeasuringMsg.header.stamp = ros::Time::now();
                    stopMeasuringPub.publish(stopMeasuringMsg);
                }
            }
            if (!running) {
                return;
            }



            // Publish the position.
            if(dogGoalVizPub.getNumSubscribers() > 0) {
                std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
                geometry_msgs::PointStamped goalPoint;
                goalPoint.point.x = goalPosition.x;
                goalPoint.point.y = goalPosition.y;
                goalPoint.point.z = goalPosition.z;
                goalPoint.header.stamp = ros::Time(currTime.Double());
                goalPoint.header.frame_id = "/map";
                dogGoalVizPub.publish(
                        utils::createMarker(goalPoint.point, goalPoint.header, RED, true));
            }

            // Calculate current errors
            const math::Vector3 worldPose = this->model->GetWorldPose().pos;
            const double errorX = calcError(worldPose.x, goalPosition.x);
            const double errorY = calcError(worldPose.y, goalPosition.y);

            const common::Time deltat = currTime - this->previousTime;

            const double errorDerivativeX = calcErrorDerivative(this->previousErrorX, errorX,
                    deltat);
            const double errorDerivativeY = calcErrorDerivative(this->previousErrorY, errorY,
                    deltat);

            const double outputX = calcPDOutput(errorX, errorDerivativeX);
            const double outputY = calcPDOutput(errorY, errorDerivativeY);

            this->forceX += outputX;
            this->forceY += outputY;

            // This should only happen when the leash is binding.
            if (abs(this->forceX) > MAXIMUM_FORCE) {
                ROS_DEBUG("Maximum X force applied to force: %f", this->forceX);
                this->forceX = copysign(MAXIMUM_FORCE, this->forceX);
            }
            // Save the current error for the next iteration.
            this->previousErrorX = errorX;

            if (abs(this->forceY) > MAXIMUM_FORCE) {
                ROS_DEBUG("Maximum Y force applied to force: %f", this->forceY);
                this->forceY = copysign(MAXIMUM_FORCE, this->forceY);
            }

            // Save the current error for the next iteration.
            this->previousErrorY = errorY;

            // Save the previous time.
            this->previousTime = currTime;


            // Add force to push dog away from base of robot.
            // y = 2 * MAX_FORCE * 1/0.25^2 * (0.25 - x).^2
            const physics::ModelPtr robot = this->model->GetWorld()->GetModel("pr2");

            double avoidanceForceX = 0;
            double avoidanceForceY = 0;
            if(robot){
                math::Vector3 basePosition = robot->GetLink("base_footprint")->GetWorldPose().pos;
                basePosition.z = 0;
                math::Vector3 dogPose = this->model->GetWorldPose().pos;
                dogPose.z = 0;
                double distance = dogPose.Distance(basePosition);
                if(distance - BASE_RADIUS < MIN_DISTANCE_FROM_ROBOT){
                    double avoidanceForce = AVOIDANCE_FORCE_MULT * MAXIMUM_FORCE * 1.0
                            / utils::square(MIN_DISTANCE_FROM_ROBOT)
                    * utils::square(MIN_DISTANCE_FROM_ROBOT - max(distance - BASE_RADIUS, 0.0));
                    math::Vector3 diff = (this->model->GetWorldPose().pos - basePosition).Normalize();
                    avoidanceForceX = avoidanceForce * diff.x;
                    avoidanceForceY = avoidanceForce * diff.y;
                    // ROS_INFO("distance %f BX %f BY %f AX %f AY %f", distance, this->forceX, this->forceY, avoidanceForceX, avoidanceForceY);
                }
            }

            this->appliedForceX = this->forceX + avoidanceForceX;
            this->appliedForceY = this->forceY + avoidanceForceY;

            if (abs(this->appliedForceX) > AVOIDANCE_FORCE_MULT * MAXIMUM_FORCE) {
                this->appliedForceX = copysign(AVOIDANCE_FORCE_MULT * MAXIMUM_FORCE,
                        this->appliedForceX);
            }
            if (abs(this->appliedForceY) > AVOIDANCE_FORCE_MULT * MAXIMUM_FORCE) {
                this->appliedForceY = copysign(AVOIDANCE_FORCE_MULT * MAXIMUM_FORCE,
                        this->appliedForceY);
            }
        }

        // Ensure the dog didn't get lifted. Can't apply force if it did. Apply a smoothing function
        // such that there is 100% traction at 0.05 height and 0% traction at 0.2 height.
        double liftFactor = min(log(10 * this->model->GetWorldPose().pos.z) / log(10 * 0.05),
                1.0);

        body->AddForce(math::Vector3(this->appliedForceX * liftFactor, this->appliedForceY * liftFactor, 0.0));

        // Calculate the torque
        const math::Vector3 relativeForce = body->GetRelativeForce();
        this->forceZ = atan2(relativeForce.y, relativeForce.x) / (3 * pi);

        body->AddRelativeTorque(math::Vector3(0.0, 0.0, this->forceZ));
    }

private:
    static double calcError(const double _currentPosition, const double _goalPosition) {
        return _goalPosition - _currentPosition;
    }

    static double calcErrorDerivative(const double _previousError, const double _currentError,
            const common::Time& _deltaTime) {
        return (_currentError - _previousError) / _deltaTime.Double();
    }

    double calcPDOutput(const double _error, const double _errorDerivative) {
        return KP * _error + KD * _errorDerivative;
    }

    math::Vector3 calcGoalPosition(common::Time time, bool& running, bool& ended) {
        dogsim::GetPath getPath;
        getPath.request.time = ros::Time(time.Double());
        getPathClient.call(getPath);
        if (!getPath.response.started || getPath.response.ended) {
            ended = getPath.response.ended;
            running = false;
            return math::Vector3();
        }

        running = true;
        ended = false;

        // Check the goal for the current time.
        gazebo::math::Vector3 base;
        base.x = getPath.response.point.point.x;
        base.y = getPath.response.point.point.y;
        base.z = getPath.response.point.point.z;

        // Gaussian function is tuned for input = [1:700]
        math::Vector3 result = addGaussians(base, this->previousBase,
                getPath.response.elapsedTime.sec);
        this->previousBase = base;
        return result;
    }

    math::Vector3 addGaussians(const math::Vector3& base, const math::Vector3& previousBase,
            const double t) {

        // Start with the base vector
        math::Vector3 result = base;

        // Calculate the derivative
        double dx = 0;
        double dy = 0;
        if (t > 0) {
            dx = base.x - previousBase.x;
            dy = base.y - previousBase.y;
        }

        // Calculate the normal
        double rx = dy;
        double ry = -dx;

        // Calculate the unit vector.
        double rl = sqrt(utils::square(rx) + utils::square(ry));
        double rux;
        double ruy;
        if (rl > 0) {
            rux = rx / rl;
            ruy = ry / rl;
        }
        else {
            rux = rx;
            ruy = ry;
        }

        // Iterate over all gaussians.
        for (unsigned int i = 0; i < gaussParams.size(); ++i) {
            // Start times are strictly increasing.
            if (t < gaussParams[i].startTime) {
                break;
            }

            double gx = t - gaussParams[i].startTime - GAUSS_HALF_WIDTH * gaussParams[i].c;
            double gy = gaussParams[i].a
                    * exp(-(utils::square(gx) / (2 * utils::square(gaussParams[i].c))));

            // Calculate the normal vector.
            double nx = rux * gy;
            double ny = ruy * gy;

            // Add it to the result.
            result.x += nx;
            result.y += ny;
        }
        return result;
    }

    //! ROS node handle
    ros::NodeHandle nh;

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Previous iteration's time
    common::Time previousTime;

    // Previous iteration's error in X
    double previousErrorX;

    // Previous iteration's error in Y
    double previousErrorY;

    // X Force
    double forceX;

    // Y Force
    double forceY;

    // Z Force
    double forceZ;

    double appliedForceX;
    double appliedForceY;

    // Previous goal
    math::Vector3 previousBase;

    struct GaussParams {
        double a;
        double c;
        double startTime;
    };

    ros::ServiceClient getPathClient;

    ros::ServiceServer dogOrientationService;

    //! Publishers for starting and stopping measurement.
    ros::Publisher startMeasuringPub;
    ros::Publisher stopMeasuringPub;

    physics::LinkPtr body;

    // Parameters for all operating gaussians
    vector<GaussParams> gaussParams;

    // Pseudo random number generator. This will always use the same seed
    // so that every run is the same.
    boost::mt19937 rng;

    ros::Publisher dogGoalVizPub;

    // KP term
    double KP;

    // KD term
    double KD;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DogModelPlugin);
}
