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

namespace {

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef vector<cv::Point> Contour;

static const double BASE_WIDTH = 0.668;
static const double PATH_LENGTH = 5.0;

static cv::Point3d point32ToCV(const geometry_msgs::Point32& p){
    cv::Point3d r;
    r.x = p.x;
    r.y = p.y;
    r.z = p.z;
    return r;
}

static geometry_msgs::Point32 createPoint(double x, double y, double z){
    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

class PathVisibilityMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;

    ros::Time lastTime;
    ros::Time startTime;

    bool debug;

    double totalScore;

    auto_ptr<Subscriber<CameraInfo> > leftCameraSub;
    auto_ptr<Subscriber<CameraInfo> > rightCameraSub;

    auto_ptr<TimeSynchronizer<CameraInfo, CameraInfo> > cameraInfoSub;
    Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;

public:
    PathVisibilityMeasurer() :
            pnh("~"), debug(true), totalScore(0), startMeasuringSub(nh, "start_measuring", 1), stopMeasuringSub(nh,
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
        ROS_INFO("Received a message @ %f", ros::Time::now().toSec());
        // Note: camera info timestamps are incorrect
        ros::Time messageTime = ros::Time::now();
        ros::Duration timePassed = messageTime - lastTime;
        ROS_INFO("Time passed: %f", timePassed.toSec());

        image_geometry::StereoCameraModel cameraModel;
        cameraModel.fromCameraInfo(lCameraInfo, rCameraInfo);

        sensor_msgs::ImageConstPtr image;

        if(debug){
            ROS_INFO("In debug mode - waiting for image");
            image = ros::topic::waitForMessage<sensor_msgs::Image>("/wide_stereo/left/image_rect", nh, ros::Duration(30.0));
            ROS_INFO("Image aquired");
            cv::namedWindow("Path Visibility", 1);
        }

        // Create the contour in the base footprint frame.
        PointCloudPtr pathRectInBaseFrame(new PointCloud());
        pathRectInBaseFrame->header.frame_id = "/base_footprint";
        pathRectInBaseFrame->header.stamp = messageTime;
        pathRectInBaseFrame->points.resize(4);
        pathRectInBaseFrame->points[0] = createPoint(1 + BASE_WIDTH / 2.0, -BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[1] = createPoint(1 + BASE_WIDTH / 2.0, BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[2] = createPoint(PATH_LENGTH + BASE_WIDTH / 2.0, BASE_WIDTH / 2.0, 0);
        pathRectInBaseFrame->points[3] = createPoint(PATH_LENGTH + BASE_WIDTH / 2.0, -BASE_WIDTH / 2.0, 0);

        // Convert to base link frame.
        PointCloudPtr pathRectInImageFrame(new PointCloud());
        try {
            tf.transformPointCloud(cameraModel.tfFrame(), ros::Time(0), *pathRectInBaseFrame, pathRectInBaseFrame->header.frame_id, *pathRectInImageFrame);
        }
        catch(tf::TransformException& e){
            ROS_ERROR("Failed to transform from /base_footprint to %s", cameraModel.tfFrame().c_str());
            return;
        }

        // Raytrace the corners of the image
        vector<cv::Point> corners(5);
        for(unsigned int i = 0; i < pathRectInImageFrame->points.size(); ++i){
            ROS_INFO("Point: x %f y %f z %f", pathRectInImageFrame->points[i].x, pathRectInBaseFrame->points[i].y, pathRectInBaseFrame->points[i].z);
        }

        // TODO: Not sure about the left/right here.
        corners[0] = cameraModel.left().project3dToPixel(point32ToCV(pathRectInImageFrame->points[0]));
        corners[1] = cameraModel.left().project3dToPixel(point32ToCV(pathRectInImageFrame->points[1]));
        corners[2] = cameraModel.left().project3dToPixel(point32ToCV(pathRectInImageFrame->points[2]));
        corners[3] = cameraModel.left().project3dToPixel(point32ToCV(pathRectInImageFrame->points[3]));
        corners[4] = corners[0]; // Close the loop

        for(unsigned int i = 0; i < corners.size(); ++i){
            ROS_INFO("Pixel: x %i y %i", corners[i].x, corners[i].y);
        }
        // Create contour of the image rect
        Contour imageRect(5);
        imageRect[0] = cv::Point(0, 0);
        imageRect[1] = cv::Point(cameraModel.left().fullResolution().width, 0);
        imageRect[2] = cv::Point(cameraModel.left().fullResolution().width, cameraModel.left().fullResolution().height);
        imageRect[3] = cv::Point(0, cameraModel.left().fullResolution().height);
        imageRect[4] = imageRect[0]; // Close the loop
        for(unsigned int i = 0; i < imageRect.size(); ++i){
            ROS_INFO("Image rect pixel: x %i y %i", imageRect[i].x, imageRect[i].y);
        }

        if(debug){
            cv_bridge::CvImagePtr share = cv_bridge::toCvCopy(image);

            // draw the square as a closed polyline
            vector<Contour> contours(2);
            contours[0] = corners;
            contours[1] = imageRect;
            cv::drawContours(share->image, contours, -1, CV_RGB(0, 255, 0), 3, CV_AA);

            // show the resultant image
            cv::imshow("Path Visibility", share->image);
            cv::waitKey(0);
        }

        // Now calculate the area
        Contour result;
        double intersectingArea = cv::intersectConvexConvex(imageRect, corners, result, true);
        ROS_INFO("Intersecting area: %f path area: %f: ", intersectingArea, cv::contourArea(corners));
        // Update the score
        ROS_INFO("Incrementing total score by %f", timePassed.toSec() * intersectingArea / cv::contourArea(corners));
        totalScore += timePassed.toSec() * intersectingArea / cv::contourArea(corners);
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
