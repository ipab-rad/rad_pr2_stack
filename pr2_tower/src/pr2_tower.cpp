/**
 * @file      pr2_tower.cpp
 * @brief     Launches pick place action server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-09-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_tower/tower_sm.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_tower");
  ros::NodeHandle nh("~");
  // ros::AsyncSpinner spinner(4);
  // spinner.start();

  if (!ros::param::has("/pr2_tower/num_blocks")) {
    ROS_ERROR("Cannot read pr2_tower parameters!");
    ros::shutdown();
  }

  TowerSM tower_sm(nh, "build_tower");
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
