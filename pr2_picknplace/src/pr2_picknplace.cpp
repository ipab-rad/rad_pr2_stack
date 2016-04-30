/**
 * @file      pr2_picknplace.cpp
 * @brief     Launches pick place action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_picknplace/pick_place.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_picknplace");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string arm;
  if (!ros::param::get("arm", arm)) {
    ROS_WARN("[PICKPLACE] Arm was not chosen! Using left as default");
    arm = "left_arm";
  }
  PickPlaceAction pick_place(nh, "pick_place", arm);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
