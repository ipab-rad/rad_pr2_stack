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
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string arm = argv[1];
  // for (int i = 0; i < argc; i++)
  // {ROS_INFO_STREAM("argv[" << i << "] = " << argv[i]); }
  // ROS_INFO_STREAM("NS: " << ros::this_node::getNamespace());
  // if (!ros::param::get("/pr2_picknplace_right/arm", arm)) {
  //   ROS_ERROR("Cannot read parameters!");
  //   ros::shutdown();
  // }


  PickPlaceAction pick_place(nh, "pr2_picknplace", arm);
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
