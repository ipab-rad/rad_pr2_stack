#include <ros/ros.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "pr2_eye_track_demo/box_delivery.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pr2_eye_track_demo");
    ros::NodeHandle n("~");

    std::string frame_names[] = {"tag_22"}; // TODO: Move to param
    BoxDelivery box_delivery(&n);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate r(30);
    while (ros::ok()) {
        ROS_INFO_THROTTLE(60, "Waiting!");
        ros::spinOnce();

        for (auto fr : frame_names) {
            geometry_msgs::TransformStamped tfStamped;
            try {
                tfStamped = tfBuffer.lookupTransform("base_link", fr, ros::Time(0));
                box_delivery.updatePose(fr, tfStamped);
                ROS_INFO_STREAM_THROTTLE(1, "Updated pose for " << fr << ".");
                ROS_DEBUG_STREAM("TF for " << fr << std::endl << tfStamped);
            } catch (tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(1, "(throttled) Error finding tf. E: %s", ex.what());
                continue;
            }
        }

        // switch (etg_roi_) {
        //     case 0: // Assembling arm
        //         GiveArmBolts();
        //         break;
        //     case 1: // Putting plate spacers
        //         GivePlateSpacers();
        //         // GivePlateSpacersAndWrench();
        //         break;
        //     case 2: // Putting plate
        //         GivePlateBoltsAndHold();
        //         break;
        //     case 3: // Putting laser on the right
        //         GiveCableRight();
        //         break;
        //     case 4: // Putting laser on the left
        //         GiveCableLeft();
        //         break;
        //     default:
        //         ros::spinOnce();
        //         rate_.sleep();
        // }

        r.sleep();
    }

    ros::shutdown();
    return 0;
}
