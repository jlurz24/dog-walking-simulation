#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>

namespace {
    class MapBroadcaster {
        private:
            tf::TransformBroadcaster br;
            ros::NodeHandle nh;
            tf::TransformListener tf;
            ros::Timer driver;
            ros::ServiceClient modelStateClient;
        public:
            MapBroadcaster(){
                driver = nh.createTimer(ros::Duration(0.1), &MapBroadcaster::callback, this);
                modelStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persist */);
            }
        
        void callback(const ros::TimerEvent& event){
        
            // Lookup the true location.
            gazebo_msgs::GetModelState getModelState;
            getModelState.request.model_name = "pr2";
            modelStateClient.call(getModelState);
            if(!getModelState.response.success){
                ROS_ERROR("get_model_state failed");
                return;
            }
        
            tf::Vector3 trueTranslation = tf::Vector3(getModelState.response.pose.position.x, getModelState.response.pose.position.y, getModelState.response.pose.position.z);
            tf::Quaternion trueOrientation = tf::Quaternion(getModelState.response.pose.orientation.w, getModelState.response.pose.orientation.x, getModelState.response.pose.orientation.y, getModelState.response.pose.orientation.z);
        
            // Lookup the transform from odom_combined to the base footprint.
            geometry_msgs::PoseStamped baseInOdomFrame;
            geometry_msgs::PoseStamped base;
            base.header.frame_id = "base_footprint";
            base.header.stamp = event.current_real;
            tf::quaternionTFToMsg(tf::createIdentityQuaternion(), base.pose.orientation);
        
            tf.waitForTransform("odom_combined", "base_footprint", event.current_real, ros::Duration(0.5));
            try {
                tf.transformPose("odom_combined", event.current_real, base, "base_footprint", baseInOdomFrame);
            }
            catch(tf::TransformException& ex){
                ROS_INFO("Failed to transform /base_footprint to /odom_combined %s", ex.what());
                return;
            }
        
            // Convert to a TF vector
            tf::Vector3 odomTranslation(baseInOdomFrame.pose.position.x, baseInOdomFrame.pose.position.y, baseInOdomFrame.pose.position.z);
            tf::Quaternion odomOrientation(baseInOdomFrame.pose.orientation.w, baseInOdomFrame.pose.orientation.x, baseInOdomFrame.pose.orientation.y, baseInOdomFrame.pose.orientation.z);
        
            // Now create a transform that is the difference between the two.
            tf::Transform transform;
            transform.setOrigin(trueTranslation - odomTranslation);
            transform.setRotation(odomOrientation.inverse() * trueOrientation);
            br.sendTransform(tf::StampedTransform(transform, event.current_real, "map", "odom_combined"));
        }
    };
}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_broadcaster");
  MapBroadcaster broadcaster;
  ros::spin();
  return 0;
};
