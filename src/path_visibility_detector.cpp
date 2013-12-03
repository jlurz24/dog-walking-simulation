#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2/LinearMath/btVector3.h>
#include <dogsim/ViewChangeRequest.h>
#include <dogsim/PathViewMetrics.h>

namespace {

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace dogsim;

typedef vector<cv::Point> Contour;

static const double VIEW_RATIO_THRESHOLD_DEFAULT = 0.99;
static const double CHANGE_PENDING_DELAY_DEFAULT = 0.5;
static const double VIEW_WIDTH_DEFAULT = 0.668;
static const double VIEW_LENGTH_DEFAULT = 3.0;
static const double VIEW_OFFSET_DEFAULT = VIEW_WIDTH_DEFAULT / 2.0;

static geometry_msgs::Point32 createPoint(double x, double y, double z){
    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

static vector<cv::Point2f> to2DPoints(const vector<geometry_msgs::Point32>& points, const cv::Point3d& centerPoint){
    vector<cv::Point2f> results(points.size());
    for(unsigned int i = 0; i < points.size(); ++i){
        const btVector3 pVector = btVector3(points[i].x, points[i].y, points[i].z);
        // The points have varying z values. We need to ray trace back to the focal point
        // of the lens to distance Z=1 to match the raytraced values from the image.
        const btVector3 focal = btVector3(centerPoint.x, centerPoint.y, centerPoint.z);
        const btVector3 focalToP = (pVector - focal).normalized();
        const btScalar angle = focalToP.angle(btVector3(0, 0, 1));
        const btScalar length = btScalar(1) / btCos(angle);

        const btVector3 adjusted = focal + length * focalToP;

        results[i] = cv::Point2f(adjusted.x(), adjusted.y());
    }
    return results;
}

static vector<cv::Point2f> to2DPoints(const vector<cv::Point3d>& points){
    vector<cv::Point2f> results(points.size());
    for(unsigned int i = 0; i < points.size(); ++i){
        results[i] = cv::Point2f(points[i].x, points[i].y);
    }
    return results;
}

class PathVisibilityDetector {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;
    bool viewChangeRequestPending;
    ros::Timer changeRequestTimer;
    double viewRatioThreshold;
    double viewWidth;
    double viewLength;
    double viewOffset;
    ros::Duration changePendingDelay;
    auto_ptr<Subscriber<CameraInfo> > leftCameraSub;

    ros::Publisher pathViewMetricsPub;
    ros::Publisher viewChangeRequestPub;
public:
    PathVisibilityDetector() :
            pnh("~"),
            viewChangeRequestPending(false){
        ROS_DEBUG("Initializing path visibility detector");
        double changePendingDelayD;
        pnh.param<double>("change_pending_delay", changePendingDelayD, CHANGE_PENDING_DELAY_DEFAULT);
        changePendingDelay.fromSec(changePendingDelayD);

        pnh.param<double>("view_ratio_threshold", viewRatioThreshold, VIEW_RATIO_THRESHOLD_DEFAULT);
        pnh.param<double>("view_length", viewLength, VIEW_LENGTH_DEFAULT);
        pnh.param<double>("view_width", viewWidth, VIEW_WIDTH_DEFAULT);
        pnh.param<double>("view_offset", viewOffset, VIEW_OFFSET_DEFAULT);

        leftCameraSub.reset(new Subscriber<CameraInfo>(nh, "/wide_stereo/left/camera_info", 1));
        leftCameraSub->registerCallback(boost::bind(&PathVisibilityDetector::callback, this, _1));
        viewChangeRequestPub = nh.advertise<ViewChangeRequest>("/dogsim/view_change_request", 1);
        pathViewMetricsPub = nh.advertise<PathViewMetrics>("/path_visibility_detector/metrics", 1);
        ROS_DEBUG("Path visibility detector initialization complete");
    }

private:
    void timeoutCallback(const ros::TimerEvent& e){
        ROS_INFO("Timeout fired. Publishing view change request.");
        ViewChangeRequestPtr viewChangeRequest(new ViewChangeRequest());
        viewChangeRequest->header.stamp = e.current_real;
        viewChangeRequest->header.frame_id = "/base_footprint";
        viewChangeRequest->center.point.y = viewWidth / 2.0;
        viewChangeRequest->center.point.x = viewOffset + viewLength / 2.0;
        viewChangeRequest->center.point.z = 0;
        viewChangeRequest->center.header = viewChangeRequest->header;
        viewChangeRequestPub.publish(viewChangeRequest);
        viewChangeRequestPending = false;
    }

    void callback(const CameraInfoConstPtr& lCameraInfo) {
        ROS_DEBUG("Received a camera info message @ %f", ros::Time::now().toSec());

        // Note: camera info timestamps are incorrect
        ros::Time messageTime = ros::Time::now();

        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(lCameraInfo);

        // Create the contour in the base footprint frame.
        PointCloudPtr pathRectInBaseFrame(new PointCloud());
        pathRectInBaseFrame->header.frame_id = "/base_footprint";
        pathRectInBaseFrame->header.stamp = messageTime;
        pathRectInBaseFrame->points.resize(5);
        pathRectInBaseFrame->points[0] = createPoint(viewOffset / 2.0, -viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[1] = createPoint(viewOffset / 2.0, viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[2] = createPoint(viewLength + viewOffset / 2.0, viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[3] = createPoint(viewLength + viewOffset / 2.0, -viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[4] = pathRectInBaseFrame->points[0];

        // Convert to image frame.
        PointCloudPtr pathRectInImageFrame(new PointCloud());
        tf.waitForTransform(cameraModel.tfFrame(), pathRectInBaseFrame->header.frame_id, ros::Time(0), ros::Duration(5.0));
        try {
            tf.transformPointCloud(cameraModel.tfFrame(), ros::Time(0), *pathRectInBaseFrame, pathRectInBaseFrame->header.frame_id, *pathRectInImageFrame);
        }
        catch(tf::TransformException& e){
            ROS_WARN("Failed to transform from /base_footprint to %s", cameraModel.tfFrame().c_str());
            return;
        }

        // Create contour of the image rect
        Contour imageRect(5);
        imageRect[0] = cv::Point(0, 0);
        imageRect[1] = cv::Point(cameraModel.fullResolution().width, 0);
        imageRect[2] = cv::Point(cameraModel.fullResolution().width, cameraModel.fullResolution().height);
        imageRect[3] = cv::Point(0, cameraModel.fullResolution().height);
        imageRect[4] = imageRect[0]; // Close the loop

        vector<cv::Point3d> imageRectInCameraFrame(imageRect.size());
        for(unsigned int i = 0; i < imageRect.size(); ++i){
            // Raytrace into 3d points.
            imageRectInCameraFrame[i] = cameraModel.projectPixelTo3dRay(imageRect[i]);
        }

        // PR2 wide camera lens is has a 2.5mm focal length.
        cv::Point3d centerPoint = cameraModel.projectPixelTo3dRay(cv::Point(cameraModel.cx(), cameraModel.cy()));
        centerPoint.z = -0.0025;

        vector<cv::Point2f> pathPointsIn2D = to2DPoints(pathRectInImageFrame->points, centerPoint);
        vector<cv::Point2f> imagePointsIn2D = to2DPoints(imageRectInCameraFrame);

        // Now calculate the area
        Contour result;
        double intersectingArea = cv::intersectConvexConvex(imagePointsIn2D, pathPointsIn2D, result, false);
        double pathArea = cv::contourArea(pathPointsIn2D, false);

        ROS_INFO("Intersecting area: %f path area: %f:", intersectingArea, pathArea);

        double visibleRatio = intersectingArea / pathArea;
        if(visibleRatio < viewRatioThreshold){
            if(!viewChangeRequestPending){
                ROS_INFO("Starting the view change request timer");
                viewChangeRequestPending = true;
                nh.createTimer(changePendingDelay, &PathVisibilityDetector::timeoutCallback, this,
                                true /* One shot */);
            }
            else {
                ROS_DEBUG("View change request timer already started");
            }
        }
        else if(viewChangeRequestPending){
            viewChangeRequestPending = false;
            changeRequestTimer.stop();
        }

        // Now fire the event for metrics
        PathViewMetricsPtr metrics(new PathViewMetrics());
        metrics->header.stamp = messageTime;
        metrics->ratio = visibleRatio;
        pathViewMetricsPub.publish(metrics);
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visibility_detector");
    PathVisibilityDetector pvd;
    ros::spin();
    return 0;
}
