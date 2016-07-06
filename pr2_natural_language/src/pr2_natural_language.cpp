/**
 * @file      pr2_natural_language.cpp
 * @brief     Launches pick place action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_natural_language/head.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_natural_language");
  ros::NodeHandle nh("~");

  Head head(nh);
  ros::Rate r(5);

  head.ready();
  // head.shake(2);
  while (ros::ok()) {
    ros::spinOnce();
    head.update();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
