/**
 * @file      pr2_softness_sorter.cpp
 * @brief     Launches grasp manager code
 * @author    Daniel Angelov <d.angelov@sms.ed.ac.uk>
 * @date      2016-06-08
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <pr2_grasp_manager/grasp_manager.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_softness_sorter");
  ros::NodeHandle nh("~");

  GraspManager grasp_manager(nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    if (grasp_manager.getGrasp()) {
      ROS_INFO("Attempting to pick up object!");
      if (grasp_manager.sendPick()) {
        ROS_INFO("Placing the object!");
        double hardness = grasp_manager.isSoft();
        ROS_WARN_STREAM("hardness: " << hardness);
        if (hardness > 0.17) {
          ROS_WARN("It is HARD!");
          grasp_manager.sendPlace(PLACE_LOCATION::HARD);
        } else {
          ROS_WARN("It is soft");
          grasp_manager.sendPlace(PLACE_LOCATION::SOFT);
        }
      } else {
        ROS_INFO("Moving out of the way!");
        grasp_manager.sendMoveTo();
      }
    }
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
