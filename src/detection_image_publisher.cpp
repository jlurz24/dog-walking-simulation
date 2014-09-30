#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <dogsim/DogPosition.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
namespace {
using namespace std;
using namespace dogsim;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, DogPosition, sensor_msgs::CameraInfo> CameraDogSyncPolicy;
typedef message_filters::Synchronizer<CameraDogSyncPolicy> CameraDogSync;

class DetectionImagePublisher {
private:

    //! The node handle we'll be using
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    // Publisher for the resulting image.
    ros::Publisher detectedImagePub;

    // TEMP
    ros::Publisher dogInBasePub;
    ros::Publisher dogInImagePub;

    //! Frame transformer
    tf::TransformListener tf;

    //! Image subscriber
    auto_ptr<message_filters::Subscriber<sensor_msgs::Image> > imageSub;

    //! Dog position subscriber
    auto_ptr<message_filters::Subscriber<DogPosition> > dogPositionSub;

    //! Subscription for the camera info
    auto_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > cameraSub;

    auto_ptr<CameraDogSync> sync;

    double dogLength;

    static const double DOG_LENGTH_DEFAULT = 0.25;

public:
    //! ROS node initialization
    DetectionImagePublisher() :
        pnh("~") {

        ros::SubscriberStatusCallback connectCB = boost::bind(
                &DetectionImagePublisher::startListening, this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &DetectionImagePublisher::stopListening, this);

        detectedImagePub = nh.advertise<sensor_msgs::Image>("/detection_image", 1, connectCB,
                disconnectCB);

        nh.param<double>("dog_length", dogLength, DOG_LENGTH_DEFAULT);
    }

private:

    void stopListening() {
        if (detectedImagePub.getNumSubscribers() != 0) {
            return;
        }

        ROS_DEBUG("Stopping listeners for DetectionImagePublisher");
        cameraSub->unsubscribe();
        imageSub->unsubscribe();
        dogPositionSub->unsubscribe();
    }

    void startListening() {
        if (detectedImagePub.getNumSubscribers() != 1) {
            return;
        }

        ROS_DEBUG("Starting listeners for DetectionImagePublisher");

        if(cameraSub.get() == NULL){
            cameraSub.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "camera_info_in", 1));
        }
        else {
            cameraSub->subscribe();
        }


        if (dogPositionSub.get() == NULL) {
            dogPositionSub.reset(
                    new message_filters::Subscriber<dogsim::DogPosition>(nh,
                            "dog_position_in", 1));
        }
        else {
            dogPositionSub->subscribe();
        }

        // List for the depth messages
        if (imageSub.get() == NULL) {
            imageSub.reset(
                    new message_filters::Subscriber<sensor_msgs::Image>(nh, "image_in", 1));
        }
        else {
            imageSub->subscribe();
        }

        if (sync.get() == NULL) {
            // Sync the two messages
            sync.reset(
                    new CameraDogSync(CameraDogSyncPolicy(30),
                            *imageSub, *dogPositionSub, *cameraSub));

            sync->registerCallback(boost::bind(&DetectionImagePublisher::callback, this, _1, _2, _3));
        }
    }

    void callback(const sensor_msgs::ImageConstPtr& image, const DogPositionConstPtr dogPosition, const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
        ROS_DEBUG("Received sync message");

        // Check that the camera info and image match.
        assert(cameraInfo->header.frame_id == image->header.frame_id);

        // Convert the camera frame location to the pixel location.
        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(*cameraInfo);

        // Convert to a CV image
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

        if(!dogPosition->unknown){
            geometry_msgs::PointStamped position;
            position.header = dogPosition->header;
            position.point = dogPosition->pose.pose.position;

            // Convert the dog position to the camera frame
            geometry_msgs::PointStamped dogInCameraFrame;
            dogInCameraFrame.header.frame_id = cameraModel.tfFrame();
            dogInCameraFrame.header.stamp = dogPosition->header.stamp;
            tf.waitForTransform(cameraModel.tfFrame(), dogPosition->header.frame_id,
                    dogPosition->header.stamp, ros::Duration(0.5));
            try {
                tf.transformPoint(cameraModel.tfFrame(), dogPosition->header.stamp, position, dogPosition->header.frame_id, dogInCameraFrame);
            }
            catch (tf::TransformException& e) {
                ROS_WARN("Failed to transform from %s to %s due to: %s",
                        position.header.frame_id.c_str(), cameraModel.tfFrame().c_str(), e.what());
                return;
            }

            // Convert to pixel in the image
            const cv::Point2d pixel = cameraModel.project3dToPixel(cv::Point3d(dogInCameraFrame.point.x, dogInCameraFrame.point.y, dogInCameraFrame.point.z));
            ROS_DEBUG("Pixel coordinates: %f %f", pixel.x, pixel.y);

            double pixels = dogLength * 0.5 * cameraModel.getDeltaV(dogInCameraFrame.point.y, dogInCameraFrame.point.z);
            ROS_DEBUG("Width of the dog is %f pixels. Dog length %f.", pixels, dogLength);

            // Add a circle at the detected image location.
            cv::circle(cvPtr->image, pixel, 10/* ceil(pixels) */, CV_RGB(255,0,0), 5);
        }
        else {
            ROS_DEBUG("Unknown dog position in detection image");
        }
        // Output modified video stream
        detectedImagePub.publish(cvPtr->toImageMsg());
    }
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_image_publisher");

    DetectionImagePublisher publisher;
    ros::spin();
    return 0;
}
