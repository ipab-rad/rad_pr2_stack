#include <ros/ros.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "pr2_eye_track_demo/pr2_box_delivery.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pr2_eye_track_demo");
    ros::NodeHandle n;

    std::string frame_names[] = {"kinect2_tag_121", "kinect2_tag_1"}; // Move to param
    PR2BoxDelivery box_delivery(&n);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate r(30);
    while (ros::ok()) {
        ROS_INFO_THROTTLE(60, "Waiting!");
        ros::spinOnce();

        for (auto fr : frame_names) {
            geometry_msgs::TransformStamped tfStamped;
            try {
                tfStamped = tfBuffer.lookupTransform(fr, "base_link", ros::Time(0));
                box_delivery.updatePose(fr, tfStamped);
                ROS_INFO_STREAM("Updated pose for " << fr << ".");
                ROS_DEBUG_STREAM("TF for " << fr << std::endl << tfStamped);
            } catch (tf2::TransformException& ex) {
                ROS_WARN("Error finding tf. E: %s", ex.what());
                continue;
            }
        }

        r.sleep();
    }
}
