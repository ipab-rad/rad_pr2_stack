/**
 * @file      pr2_picknplace.cpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_picknplace/pick_place.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_picknplace");
  ros::NodeHandle nh("pr2_picknplace");
  // ros::AsyncSpinner spinner(2);
  // spinner.start();
  PickPlaceAction pick_place(nh, "pick_place");

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
