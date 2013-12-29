#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/btVector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <sensor_msgs/CameraInfo.h>

namespace {
using namespace geometry_msgs;
using namespace std;
using namespace pcl;

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloudXYZ::ConstPtr PointCloudXYZConstPtr;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;

static const double DOG_HEIGHT_DEFAULT = 0.1;

class ZeroHeightDepthBroadcaster {
private:
    ros::NodeHandle nh;
    tf::TransformListener tf;
    auto_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > cameraSub;
    ros::Publisher pointsPub;
    double dogHeight;

public:
    ZeroHeightDepthBroadcaster() {
        cameraSub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "camera_info", 1));
        cameraSub->registerCallback(boost::bind(&ZeroHeightDepthBroadcaster::callback, this, _1));

        // Publish depth messages
        pointsPub = nh.advertise<sensor_msgs::PointCloud2>("points", 1);
        nh.param("dog_height", dogHeight, DOG_HEIGHT_DEFAULT);
    }

    btVector3 intersection(const btVector3& n, const btVector3& p0, const btVector3& l, const btVector3& l0){
        // Make sure everything is normalized
        assert(n.length() - 1 < 1e-6);
        assert(l.length() - 1 < 1e-6);

        btScalar denom = n.dot(l);
        if(fabs(denom) < 1e-6){
          return btVector3(numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
        }

        btScalar t = n.dot(p0 - l0)/ denom;
        if(t < 0){
	  // Intersection behind the camera
          return btVector3(numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
	}
        // Now project the vector l to the plane.
        return l0 + (l * t);
    }

    void callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
        ROS_DEBUG("Received a camera info message @ %f", ros::Time::now().toSec());

        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(cameraInfo);

        // Calculate the ground normal
        Vector3Stamped groundNormalInBaseFrame;
        groundNormalInBaseFrame.header.frame_id = "/base_footprint";
        groundNormalInBaseFrame.header.stamp = cameraModel.stamp();
        groundNormalInBaseFrame.vector.x = 0;
        groundNormalInBaseFrame.vector.y = 0;
        groundNormalInBaseFrame.vector.z = 1.0;

        Vector3Stamped groundNormal;
        if(!tf.waitForTransform(cameraModel.tfFrame(), groundNormalInBaseFrame.header.frame_id, cameraModel.stamp(), ros::Duration(5))){
          ROS_WARN("Failed to get transform");
          return;
        }
        
        tf.transformVector(cameraModel.tfFrame(), groundNormalInBaseFrame, groundNormal);

        // Calculate the center point
        PointStamped groundOriginInBaseFrame;
        groundOriginInBaseFrame.header.frame_id = "/base_footprint";
        groundOriginInBaseFrame.header.stamp = cameraModel.stamp();
        groundOriginInBaseFrame.point.z = dogHeight;
        PointStamped groundOrigin;
        tf.transformPoint(cameraModel.tfFrame(), groundOriginInBaseFrame, groundOrigin);

        btVector3 groundOriginV(groundOrigin.point.x, groundOrigin.point.y, groundOrigin.point.z);
        btVector3 groundNormalV(groundNormal.vector.x, groundNormal.vector.y, groundNormal.vector.z);

        cv::Point3d cameraOriginCV = cameraModel.projectPixelTo3dRay(cv::Point2d(cameraModel.cx(), cameraModel.cy()));
        btVector3 cameraOrigin(cameraOriginCV.x, cameraOriginCV.y, 0);
        PointCloudXYZPtr cloud(new PointCloudXYZ());

        cloud->is_dense = false;
        cloud->width = cameraModel.fullResolution().width;
        cloud->height = cameraModel.fullResolution().height;
        for(int u = 0; u < cameraModel.fullResolution().width; ++u){
            for(int v = 0; v < cameraModel.fullResolution().height; ++v){
                cv::Point3d p = cameraModel.projectPixelTo3dRay(cv::Point2d(u, v));
                btVector3 pVector(p.x, p.y, p.z);
                btVector3 zeroHVector = intersection(groundNormalV, groundOriginV, pVector.normalized(), cameraOrigin);
                cloud->points.push_back(PointXYZ(zeroHVector.x(), zeroHVector.y(), zeroHVector.z()));
            }
        }

        // Convert to a ROS message and publish
        sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2());
        toROSMsg(*cloud, *output);
        ROS_DEBUG("Publishing a message with %lu points in %s frame", output->data.size(), cameraModel.tfFrame().c_str());
        output->header.frame_id = cameraModel.tfFrame();
        output->header.stamp = cameraModel.stamp();

        pointsPub.publish(output);
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zero_height_depth_broadcaster");
    ZeroHeightDepthBroadcaster broadcaster;
    ros::spin();
    return 0;
}
;
