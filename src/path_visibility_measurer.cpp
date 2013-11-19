#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/time_synchronizer.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/btVector3.h>

namespace {

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef vector<cv::Point> Contour;

static const double BASE_WIDTH = 0.668;
static const double PATH_LENGTH = 5.0;

static geometry_msgs::Point32 createPoint(double x, double y, double z){
    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

static vector<cv::Point> to2DPoints(const vector<geometry_msgs::Point32>& points, const cv::Point3d& centerPoint){
    vector<cv::Point> results(points.size());
    for(unsigned int i = 0; i < points.size(); ++i){
        const btVector3 pVector = btVector3(points[i].x, points[i].y, points[i].z);
        // The points have varying z values. We need to ray trace back to the focal point
        // of the lens to distance Z=1 to match the raytraced values from the image.
        const btVector3 focal = btVector3(centerPoint.x, centerPoint.y, centerPoint.z);
        const btVector3 focalToP = (pVector - focal).normalized();
        const btScalar angle = focalToP.angle(btVector3(0, 0, 1));
        const btScalar length = btScalar(1) / btCos(angle);

        const btVector3 adjusted = focal + length * focalToP;

        // Multiply by 1000 because the area calculation is pixel based.
        results[i] = cv::Point(adjusted.x() * 1000, adjusted.y() * 1000);
    }
    return results;
}

static vector<cv::Point> to2DPoints(const vector<cv::Point3d>& points){
    vector<cv::Point> results(points.size());
    for(unsigned int i = 0; i < points.size(); ++i){
        // Multiply by 1000 because the area calculation is pixel based.
        results[i] = cv::Point(points[i].x * 1000, points[i].y * 1000);
    }
    return results;
}

class PathVisibilityMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;

    ros::Time lastTime;
    ros::Time startTime;

    double totalScore;

    auto_ptr<Subscriber<CameraInfo> > leftCameraSub;
    auto_ptr<Subscriber<CameraInfo> > rightCameraSub;

    auto_ptr<TimeSynchronizer<CameraInfo, CameraInfo> > cameraInfoSub;
    Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;

public:
    PathVisibilityMeasurer() :
            pnh("~"), totalScore(0), startMeasuringSub(nh, "start_measuring", 1), stopMeasuringSub(nh,
                    "stop_measuring", 1) {

        leftCameraSub.reset(new Subscriber<CameraInfo>(nh, "/wide_stereo/left/camera_info", 1));
        rightCameraSub.reset(new Subscriber<CameraInfo>(nh, "/wide_stereo/right/camera_info", 1));
        cameraInfoSub.reset(new TimeSynchronizer<CameraInfo, CameraInfo>(*leftCameraSub, *rightCameraSub.get(), 10));

        startMeasuringSub.registerCallback(
                boost::bind(&PathVisibilityMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(
                boost::bind(&PathVisibilityMeasurer::stopMeasuring, this, _1));
    }

private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg) {
        startTime = lastTime = msg->header.stamp;
        cameraInfoSub->registerCallback(boost::bind(&PathVisibilityMeasurer::callback, this, _1, _2));
        ROS_INFO("Measurement initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg) {
        ROS_INFO("Measurement completed");
        cameraInfoSub.release();
        ROS_INFO("Total path visibility score was %f over %f seconds", totalScore, msg->header.stamp.toSec() - startTime.toSec());
    }

    void callback( const CameraInfoConstPtr& lCameraInfo,  const CameraInfoConstPtr& rCameraInfo) {
        ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

        // Note: camera info timestamps are incorrect
        ros::Time messageTime = ros::Time::now();
        ros::Duration timePassed = messageTime - lastTime;

        image_geometry::StereoCameraModel cameraModel;
        cameraModel.fromCameraInfo(lCameraInfo, rCameraInfo);

        // Create the contour in the base footprint frame.
        PointCloudPtr pathRectInBaseFrame(new PointCloud());
        pathRectInBaseFrame->header.frame_id = "/base_footprint";
        pathRectInBaseFrame->header.stamp = messageTime;
        pathRectInBaseFrame->points.resize(5);
        pathRectInBaseFrame->points[0] = createPoint(1 + BASE_WIDTH / 2.0, -BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[1] = createPoint(1 + BASE_WIDTH / 2.0, BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[2] = createPoint(PATH_LENGTH + BASE_WIDTH / 2.0, BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[3] = createPoint(PATH_LENGTH + BASE_WIDTH / 2.0, -BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[4] = pathRectInBaseFrame->points[0];

        // Convert to image frame.
        PointCloudPtr pathRectInImageFrame(new PointCloud());
        try {
            tf.transformPointCloud(cameraModel.tfFrame(), ros::Time(0), *pathRectInBaseFrame, pathRectInBaseFrame->header.frame_id, *pathRectInImageFrame);
        }
        catch(tf::TransformException& e){
            ROS_ERROR("Failed to transform from /base_footprint to %s", cameraModel.tfFrame().c_str());
            return;
        }

        // Create contour of the image rect
        Contour imageRect(5);
        imageRect[0] = cv::Point(0, 0);
        imageRect[1] = cv::Point(cameraModel.left().fullResolution().width, 0);
        imageRect[2] = cv::Point(cameraModel.left().fullResolution().width, cameraModel.left().fullResolution().height);
        imageRect[3] = cv::Point(0, cameraModel.left().fullResolution().height);
        imageRect[4] = imageRect[0]; // Close the loop

        vector<cv::Point3d> imageRectInCameraFrame(imageRect.size());
        for(unsigned int i = 0; i < imageRect.size(); ++i){
            // Raytrace into 3d points.
            imageRectInCameraFrame[i] = cameraModel.left().projectPixelTo3dRay(imageRect[i]);
        }

        // PR2 wide camera lens is has a 2.5mm focal length.
        cv::Point3d centerPoint = cameraModel.left().projectPixelTo3dRay(cv::Point(0, 0));
        centerPoint.z = -0.0025;

        vector<cv::Point> pathPointsIn2D = to2DPoints(pathRectInImageFrame->points, centerPoint);
        vector<cv::Point> imagePointsIn2D = to2DPoints(imageRectInCameraFrame);

        // Now calculate the area
        Contour result;
        double intersectingArea = cv::intersectConvexConvex(imagePointsIn2D, pathPointsIn2D, result, true);
        double pathArea = cv::contourArea(pathPointsIn2D);

        ROS_DEBUG("Intersecting area: %f path area: %f: ", intersectingArea, pathArea);

        // Update the score
        double increment = timePassed.toSec() * intersectingArea / pathArea;
        ROS_DEBUG("Incrementing total score by %f", increment);
        totalScore += increment;
        lastTime = messageTime;
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visibility_measurer");
    PathVisibilityMeasurer pvm;
    ros::spin();
    return 0;
}
