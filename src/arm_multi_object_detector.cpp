#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <position_tracker/DetectedObjects.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <boost/math/constants/constants.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/geometry.h>

using namespace std;
using namespace pcl;

inline Eigen::Vector3f operator-(const PointXYZ& p1, const PointXYZ& p2) {
  return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloudXYZ::ConstPtr PointCloudXYZConstPtr;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;
typedef message_filters::sync_policies::ApproximateTime<cmvision::Blobs, sensor_msgs::PointCloud2> BlobCloudSyncPolicy;
typedef message_filters::Synchronizer<BlobCloudSyncPolicy> BlobCloudSync;

class MultiObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
   
    auto_ptr<message_filters::Subscriber<cmvision::Blobs> > blobsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;
    
    double clusterDistanceTolerance;
    double voxelLeafSize;
    int minClusterSize;
    int maxClusterSize;
    
    // Publisher for the resulting position event.
    ros::Publisher pub; 

    // Publish for visualization
    ros::Publisher markerPub;
    
    auto_ptr<BlobCloudSync> sync;

 public:
    MultiObjectDetector() : privateHandle("~"){
      privateHandle.param<string>("object_name", objectName, "dog");
      ROS_DEBUG("Detecting blobs with object name %s", objectName.c_str());

      privateHandle.param<double>("cluster_distance_tolerance", clusterDistanceTolerance, 0.1);
      privateHandle.param<double>("voxel_leaf_size", voxelLeafSize, 0.0);
      privateHandle.param<int>("min_cluster_size", minClusterSize, 75);
      privateHandle.param<int>("max_cluster_size", maxClusterSize, 25000);

      // Publish the object location
      ros::SubscriberStatusCallback connectCB = boost::bind(&MultiObjectDetector::startListening, this);
      ros::SubscriberStatusCallback disconnectCB = boost::bind(&MultiObjectDetector::stopListening, this);

      pub = nh.advertise<position_tracker::DetectedObjects>("object_locations/" + objectName, 1, connectCB, disconnectCB);
      markerPub = nh.advertise<visualization_msgs::MarkerArray>("object_locations/markers", 1, connectCB, disconnectCB);
      ROS_DEBUG("Initialization of object detector complete");
    }
    
 private:
    void stopListening(){
      if(pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0){
        ROS_DEBUG("Stopping listeners for multi object detector");
        if(blobsSub.get()){
          blobsSub->unsubscribe();
        }
        if(depthPointsSub.get()){
          depthPointsSub->unsubscribe();
        }
      }
    }

    void startListening(){
      if(pub.getNumSubscribers() + markerPub.getNumSubscribers() != 1){
        return;
      }

      ROS_DEBUG("Starting to listen for blob messages");
 
      if(blobsSub.get() == NULL){
        // Listen for message from cm vision when it sees an object.
        blobsSub.reset(new message_filters::Subscriber<cmvision::Blobs>(nh, "/blobs", 1));
      }
      else {
        blobsSub->subscribe();
      }
      
      // Listen for the depth messages
      if(depthPointsSub.get() == NULL){
        depthPointsSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/points", 3));
      }
      else {
        depthPointsSub->subscribe();
      }
  
      if(sync.get() == NULL){
        // Sync the two messages
        sync.reset(new BlobCloudSync(BlobCloudSyncPolicy(10), *blobsSub, *depthPointsSub));
      
        sync->registerCallback(boost::bind(&MultiObjectDetector::finalBlobCallback, this, _1, _2));
      }
      ROS_DEBUG("Registration for blob events complete.");
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){
      ROS_DEBUG("Received a blobs message @ %f", ros::Time::now().toSec());

      // Initialize the result message
      position_tracker::DetectedObjectsPtr objects(new position_tracker::DetectedObjects);
      objects->header = depthPointsMsg->header;
      // Check if there are detected blobs.
      if(blobsMsg->blobs.size() == 0){
        ROS_DEBUG("No blobs detected");
        publish(objects);
        return;
      }

      ROS_DEBUG("Depth points frame is %s and blobsMsg frame is %s",
              depthPointsMsg->header.frame_id.c_str(), blobsMsg->header.frame_id.c_str());
      assert(depthPointsMsg->header.frame_id == blobsMsg->header.frame_id);
      PointCloudXYZ depthCloud;
      fromROSMsg(*depthPointsMsg, depthCloud);

      PointCloudXYZPtr allBlobs;
      const vector<PointIndices> blobClouds = splitBlobs(depthCloud, blobsMsg, allBlobs);

      if(blobClouds.size() == 0){
        ROS_DEBUG("No blobs to use for centroid detection");
        publish(objects);
        return;
      }

      // Iterate over each detected blob and determine its centroid.
      const string& depthPointsFrame = depthPointsMsg->header.frame_id;

      if(!tf.waitForTransform("/base_footprint", depthPointsFrame, depthPointsMsg->header.stamp, ros::Duration(5.0))){
        ROS_WARN("Transform from %s to base_footprint is not yet available", depthPointsFrame.c_str());
        return;
      }

      for(unsigned int i = 0; i < blobClouds.size(); ++i){
        Eigen::Vector4f centroid;
        compute3DCentroid(*allBlobs, blobClouds[i].indices, centroid);

        // Convert the centroid to a point stamped
        geometry_msgs::PointStamped resultPoint;
        resultPoint.header.frame_id = depthPointsFrame;
        resultPoint.header.stamp = depthPointsMsg->header.stamp;

        // Convert the centroid to a geometry msg point
        ROS_DEBUG("Computed centroid in frame %s with coordinates %f, %f, %f", depthPointsMsg->header.frame_id.c_str(), centroid[0], centroid[1], centroid[2]);

        resultPoint.point.x = centroid[0];
        resultPoint.point.y = centroid[1];
        resultPoint.point.z = centroid[2];
       
        geometry_msgs::PointStamped resultPointBaseFootprint;
        resultPointBaseFootprint.header.frame_id = "/base_footprint";
        resultPointBaseFootprint.header.stamp = depthPointsMsg->header.stamp;
        tf.transformPoint(resultPointBaseFootprint.header.frame_id, resultPoint, resultPointBaseFootprint);
        ROS_DEBUG("Transformed centroid to frame %s with coordinates %f %f %f", resultPointBaseFootprint.header.frame_id.c_str(), resultPointBaseFootprint.point.x, resultPointBaseFootprint.point.y, resultPointBaseFootprint.point.z);
        objects->positions.push_back(resultPointBaseFootprint);
     }
     publish(objects);
   }
   
   void publish(const position_tracker::DetectedObjects::ConstPtr objects){
     // Publish the markers message  
     if(markerPub.getNumSubscribers() > 0){
       publishMarkers(objects);
     }

     // Broadcast the result
     pub.publish(objects);
   }
   
   void publishMarkers(position_tracker::DetectedObjectsConstPtr objects){
     visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
     for(unsigned int i = 0; i < objects->positions.size(); ++i){
       visualization_msgs::Marker marker;
       marker.id = i;
       marker.ns = "multi_object_detector/" + nh.resolveName("/blobs");
       marker.action = visualization_msgs::Marker::ADD;
       marker.type = visualization_msgs::Marker::SPHERE;
       marker.header = objects->positions[i].header;
       marker.lifetime = ros::Duration(1);
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position = objects->positions[i].point;
       marker.pose.orientation.x = 0;
       marker.pose.orientation.y = 0;
       marker.pose.orientation.z = 0;
       marker.pose.orientation.w = 1;
       marker.color.a = 1;
       marker.color.r = 0;
       marker.color.g = 1;
       marker.color.b = 0;
       marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
       markers->markers.push_back(marker);
     }

     markerPub.publish(markers);
   }

   const vector<PointIndices> splitBlobs(const PointCloudXYZ depthCloud, const cmvision::BlobsConstPtr blobsMsg, PointCloudXYZPtr& allBlobsOut){

       // Iterate over all the blobs and create a single cloud of all points.
       // We will subdivide this blob again later.
       PointCloudXYZPtr allBlobs(new PointCloudXYZ);
       ROS_DEBUG("Blobs message has %lu blobs", blobsMsg->blobs.size());
       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            ROS_DEBUG("Skipping blob named %s as it does not match", objectName.c_str());
            continue;
          }

          ROS_DEBUG("Blob image dimensions. Left %u right %u top %u bottom %u width %u height %u", blob.left, blob.right, blob.top, blob.bottom, blobsMsg->image_width, blobsMsg->image_height);
          assert(depthCloud.points.size() == blobsMsg->image_width * blobsMsg->image_height);
          for(unsigned int j = blob.top; j <= blob.bottom; ++j){
            for(unsigned int i = blob.left; i <= blob.right; ++i){
                unsigned int index = j * blobsMsg->image_width + i;
                if(index >= depthCloud.points.size()){
                    ROS_WARN("Depth cloud does not contain point for index %u", index);
                }
                else {
                    PointXYZ point = depthCloud.points.at(index);
                    allBlobs->points.push_back(point);
                }
            }
          }
      }

      allBlobs->is_dense = false;
      allBlobs->width = allBlobs->points.size();
      allBlobs->height = 1;
  
      if(voxelLeafSize > 0){
        // Use a voxel grid to downsample the input to a 1cm grid.
        VoxelGrid<PointXYZ> vg;
        PointCloudXYZPtr allBlobsFiltered(new PointCloudXYZ);
        vg.setInputCloud(allBlobs);
        vg.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
        vg.filter(*allBlobsFiltered);
        allBlobs = allBlobsFiltered;
      }

      vector<int> removedIndices;
      pcl::removeNaNFromPointCloud(*allBlobs, *allBlobs, removedIndices);
      if(allBlobs->size() == 0){
          ROS_INFO("No remaining blob points after removing NaNs");
          return std::vector<PointIndices>();
      }
      ROS_DEBUG("Points available for blob position. Extracting clusters.");

      // Creating the KdTree was much slower than direct cluster extraction.
      std::vector<PointIndices> clusterIndices;
      EuclideanClusterExtraction<PointXYZ> ec;
      ec.setClusterTolerance(clusterDistanceTolerance);
      ec.setMinClusterSize(minClusterSize);
      ec.setMaxClusterSize(maxClusterSize);
      ec.setInputCloud(allBlobs);
      ec.extract(clusterIndices);
      ROS_DEBUG("Extracted %lu clusters from blobs", clusterIndices.size());
      allBlobsOut = allBlobs;
      return clusterIndices;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "multi_object_detector");
  MultiObjectDetector mobd;
  ros::spin();
  return 0;
}

