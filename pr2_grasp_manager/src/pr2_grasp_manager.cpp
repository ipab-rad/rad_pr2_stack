/**
 * @file      pr2_grasp_manager.cpp
 * @brief     Launches grasp manager code
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-06-08
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_grasp_manager/grasp_manager.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_grasp_manager");
  ros::NodeHandle nh("~");

  GraspManager grasp_manager(nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    grasp_manager.getGrasp();
    grasp_manager.sendPick();
    grasp_manager.sendPlace();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
