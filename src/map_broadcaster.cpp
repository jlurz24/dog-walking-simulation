#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <message_filters/subscriber.h>

namespace {
    class MapBroadcaster {
        private:
            tf::TransformBroadcaster br;
            ros::NodeHandle nh;
            tf::TransformListener tf;
            message_filters::Subscriber<gazebo_msgs::ModelStates> modelStates;
    public:
        MapBroadcaster():modelStates(nh, "gazebo/model_states", 1){
            modelStates.registerCallback(boost::bind(&MapBroadcaster::callback, this, _1));
        }
        
    void callback(const gazebo_msgs::ModelStates::ConstPtr& models){
        // First lookup the transform from odom_combined to the base footprint.
        geometry_msgs::PoseStamped baseInOdomFrame;
        geometry_msgs::PoseStamped base;
        base.header.frame_id = "base_footprint";
        base.header.stamp = ros::Time::now();
        tf::quaternionTFToMsg(tf::createIdentityQuaternion(), base.pose.orientation);
        
        try {
            tf.transformPose("odom_combined", ros::Time(0), base, "base_footprint", baseInOdomFrame);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("Failed to transform /base_footprint to /odom_combined %s", ex.what());
            return;
        }
        
        bool found = false;
        tf::Vector3 trueTranslation;
        tf::Quaternion trueOrientation;
        
        // Find the real robot position.
        for(unsigned int i = 0; i < models->name.size(); ++i){
            if(models->name[i] == "pr2"){
                trueTranslation = tf::Vector3(models->pose[i].position.x, models->pose[i].position.y, models->pose[i].position.z);
                trueOrientation = tf::Quaternion(models->pose[i].orientation.w, models->pose[i].orientation.x, models->pose[i].orientation.y, models->pose[i].orientation.z);
                found = true;
                break;
            }
        }
        assert(found);
        
        // Convert to a TF vector
        tf::Vector3 odomTranslation(baseInOdomFrame.pose.position.x, baseInOdomFrame.pose.position.y, baseInOdomFrame.pose.position.z);
        tf::Quaternion odomOrientation(baseInOdomFrame.pose.orientation.w, baseInOdomFrame.pose.orientation.x, baseInOdomFrame.pose.orientation.y, baseInOdomFrame.pose.orientation.z);
        
        // Now create a transform that is the difference between the two.
        tf::Transform transform;
        transform.setOrigin(trueTranslation - odomTranslation);
        transform.setRotation(odomOrientation.inverse() * trueOrientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_combined"));
        }
    };
}
int main(int argc, char** argv){
  ros::init(argc, argv, "map_broadcaster");
  MapBroadcaster broadcaster;
  ros::spin();
  return 0;
};
