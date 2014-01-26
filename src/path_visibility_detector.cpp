#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/btVector3.h>
#include <dogsim/PathViewInfo.h>
#include <visualization_msgs/Marker.h>
#include <dogsim/utils.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/connect_holes.h>
#include <CGAL/centroid.h>

namespace {

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace dogsim;

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Point_2<K> Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

static const double VIEW_WIDTH_DEFAULT = 0.668;
static const double VIEW_LENGTH_DEFAULT = 3.0;
static const double VIEW_OFFSET_DEFAULT = VIEW_WIDTH_DEFAULT / 2.0;
static const double PATH_VIS_THRESHOLD_DEFAULT = 0.99;

static geometry_msgs::Point32 createPoint(double x, double y, double z) {
    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

static Polygon_2 to2DPoints(const vector<geometry_msgs::Point32>& points,
        const cv::Point3d& centerPoint) {
    Polygon_2 results;
    for (unsigned int i = 0; i < points.size(); ++i) {
        const btVector3 pVector = btVector3(points[i].x, points[i].y, points[i].z);
        // The points have varying z values. We need to ray trace back to the focal point
        // of the lens to distance Z=1 to match the raytraced values from the image.
        const btVector3 focal = btVector3(centerPoint.x, centerPoint.y, centerPoint.z);
        const btVector3 focalToP = (pVector - focal).normalized();
        const btScalar angle = focalToP.angle(btVector3(0, 0, 1));
        const btScalar length = btScalar(1) / btCos(angle);

        const btVector3 adjusted = focal + length * focalToP;
        results.push_back(Point_2(adjusted.x(), adjusted.y()));
    }
    if(!results.is_simple()){
        results.clear();
    }
    else if (!results.is_counterclockwise_oriented()) {
        results.reverse_orientation();
    }
    return results;
}

static Polygon_2 to2DPoints(const vector<cv::Point3d>& points) {
    Polygon_2 results;
    for (unsigned int i = 0; i < points.size(); ++i) {
        results.push_back(Point_2(points[i].x, points[i].y));
    }
    if(!results.is_simple()){
        results.clear();
    }
    else if (!results.is_counterclockwise_oriented()) {
        results.reverse_orientation();
    }
    return results;
}

class PathVisibilityDetector {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;

    double viewWidth;
    double viewLength;
    double viewOffset;

    double pathVisibilityThreshold;

    ros::Time lastFullyVisibleTime;

    auto_ptr<Subscriber<CameraInfo> > leftCameraSub;

    ros::Publisher viewPub;
    ros::Publisher viewVizPub;

public:
    PathVisibilityDetector() :
            pnh("~") {
        ROS_DEBUG("Initializing path visibility detector");

        pnh.param<double>("view_length", viewLength, VIEW_LENGTH_DEFAULT);
        pnh.param<double>("view_width", viewWidth, VIEW_WIDTH_DEFAULT);
        pnh.param<double>("view_offset", viewOffset, VIEW_OFFSET_DEFAULT);

        nh.param("path_visibility_threshold", pathVisibilityThreshold, PATH_VIS_THRESHOLD_DEFAULT);

        leftCameraSub.reset(new Subscriber<CameraInfo>(nh, "camera_info_in", 1));
        leftCameraSub->registerCallback(boost::bind(&PathVisibilityDetector::callback, this, _1));

        viewPub = nh.advertise<PathViewInfo>("/path_visibility_detector/view", 1);
        viewVizPub = nh.advertise<visualization_msgs::Marker>("/path_visibility_detector/view_viz",
                1);
        ROS_DEBUG("Path visibility detector initialization complete");
    }

private:
    void publishView(const double visibilityRatio, const geometry_msgs::PointStamped& pointToLookAt) {
        ROS_DEBUG("Publishing view change request with ratio: %f and last fully visible time: %f", visibilityRatio, lastFullyVisibleTime.toSec());
        PathViewInfoPtr pathViewInfo(new PathViewInfo());
        pathViewInfo->header = pointToLookAt.header;
        pathViewInfo->center = pointToLookAt;
        pathViewInfo->visibilityRatio = visibilityRatio;
        pathViewInfo->measuredTime = lastFullyVisibleTime;
        viewPub.publish(pathViewInfo);
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
        pathRectInBaseFrame->points.resize(4);
        pathRectInBaseFrame->points[0] = createPoint(viewOffset / 2.0, -viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[1] = createPoint(viewOffset / 2.0, viewWidth / 2.0, 0);
        pathRectInBaseFrame->points[2] = createPoint(viewLength + viewOffset / 2.0, viewWidth / 2.0,
                0);
        pathRectInBaseFrame->points[3] = createPoint(viewLength + viewOffset / 2.0,
                -viewWidth / 2.0, 0);

        // Convert to image frame.
        PointCloudPtr pathRectInImageFrame(new PointCloud());
        tf.waitForTransform(cameraModel.tfFrame(), pathRectInBaseFrame->header.frame_id,
                ros::Time(0), ros::Duration(5.0));
        try {
            tf.transformPointCloud(cameraModel.tfFrame(), ros::Time(0), *pathRectInBaseFrame,
                    pathRectInBaseFrame->header.frame_id, *pathRectInImageFrame);
        }
        catch (tf::TransformException& e) {
            ROS_WARN("Failed to transform from /base_footprint to %s",
                    cameraModel.tfFrame().c_str());
            return;
        }

        // Create contour of the image rect
        vector<cv::Point> imageRect(4);
        imageRect[0] = cv::Point(0, 0);
        imageRect[1] = cv::Point(cameraModel.fullResolution().width, 0);
        imageRect[2] = cv::Point(cameraModel.fullResolution().width,
                cameraModel.fullResolution().height);
        imageRect[3] = cv::Point(0, cameraModel.fullResolution().height);

        vector<cv::Point3d> imageRectInCameraFrame(imageRect.size());
        for (unsigned int i = 0; i < imageRect.size(); ++i) {
            // Raytrace into 3d points.
            imageRectInCameraFrame[i] = cameraModel.projectPixelTo3dRay(imageRect[i]);
        }

        cv::Point3d centerPoint = cameraModel.projectPixelTo3dRay(
                cv::Point(cameraModel.cx(), cameraModel.cy()));
        centerPoint.z = 0;

        Polygon_2 pathPointsIn2D = to2DPoints(pathRectInImageFrame->points, centerPoint);
        Polygon_2 imagePointsIn2D = to2DPoints(imageRectInCameraFrame);

        double intersectingArea = 0;
        double pathArea = 1;
        if (!pathPointsIn2D.is_empty()) {
            // Now calculate the area
            assert(pathPointsIn2D.is_simple());
            assert(imagePointsIn2D.is_simple());
            assert(!imagePointsIn2D.is_empty());

            vector<Polygon_with_holes_2> intersectionPoints;
            CGAL::intersection(pathPointsIn2D, imagePointsIn2D,
                    std::back_inserter(intersectionPoints));

            for (unsigned int i = 0; i < intersectionPoints.size(); ++i) {
                Polygon_2 connected;
                CGAL::connect_holes(intersectionPoints[i], back_inserter(connected));
                intersectingArea += CGAL::to_double(connected.area());
            }
            pathArea = CGAL::to_double(pathPointsIn2D.area());
        }

        ROS_DEBUG("Image area width %f and height %f",
                sqrt(CGAL::to_double(CGAL::squared_distance(imagePointsIn2D[0], imagePointsIn2D[1]))),
                sqrt(CGAL::to_double(CGAL::squared_distance(imagePointsIn2D[1], imagePointsIn2D[2]))));

        double imageArea = CGAL::to_double(imagePointsIn2D.area());
        double visibleRatio = intersectingArea / pathArea;
        ROS_DEBUG("Intersecting area: %f path area: %f, image area: %f, ratio: %f", intersectingArea, pathArea,
                imageArea, visibleRatio);

        if(visibleRatio > pathVisibilityThreshold){
            lastFullyVisibleTime = messageTime;
        }

        Point_2 center;
        if(pathPointsIn2D.vertices_begin() == pathPointsIn2D.vertices_end()){
            center = *pathPointsIn2D.vertices_begin();
        }
        else {
            center = CGAL::centroid(pathPointsIn2D.vertices_begin(), pathPointsIn2D.vertices_end(), K());
        }

        geometry_msgs::PointStamped pointToLookAt;
        pointToLookAt.header.frame_id = cameraModel.tfFrame();
        pointToLookAt.header.stamp = messageTime;
        pointToLookAt.point.x = CGAL::to_double(center.x());
        pointToLookAt.point.y = CGAL::to_double(center.y());
        pointToLookAt.point.z = 1.0;
        publishView(visibleRatio, pointToLookAt);

        // Publish visualization
        if (viewVizPub.getNumSubscribers() > 0) {
            std_msgs::ColorRGBA BLUE = utils::createColor(0, 0, 1);
            visualization_msgs::Marker imageViz = createMarker(1, imagePointsIn2D,
                    pathRectInImageFrame->header, BLUE);
            viewVizPub.publish(imageViz);
            std_msgs::ColorRGBA RED = utils::createColor(1, 0, 0);
            visualization_msgs::Marker pathViz = createMarker(2, pathPointsIn2D,
                    pathRectInImageFrame->header, RED);
            viewVizPub.publish(pathViz);
        }
    }

    static visualization_msgs::Marker createMarker(unsigned int id, const Polygon_2& p,
            std_msgs::Header& header, std_msgs::ColorRGBA& color) {
        visualization_msgs::Marker marker;
        marker.points.resize(p.size());
        marker.header = header;
        marker.ns = "dogsim";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = color;
        marker.scale.x = 0.1;
        for (unsigned int i = 0; i < p.size(); ++i) {
            marker.points[i].x = CGAL::to_double(p[i].x());
            marker.points[i].y = CGAL::to_double(p[i].y());
            marker.points[i].z = 1.0;
        }
        // Close the loop
        marker.points.push_back(marker.points[0]);
        return marker;
    }
};
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_visibility_detector");
    PathVisibilityDetector pvd;
    ros::spin();
    return 0;
}
