/**
 * @file      pick_place_actest.cpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-10
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_picknplace/pick_place.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_picknplace_actest");
  ros::NodeHandle nh("~");

  if (argc != 2) {ROS_ERROR("Please input number of cubes (min. 1)"); return 1;}

  actionlib::SimpleActionClient<pr2_picknplace_msgs::PickPlaceAction>
  ac("/pr2_picknplace/pick_place", true);

  ac.waitForServer();

  // Variables
  int n_cubes = atoi(argv[1]);

  if (n_cubes < 1) {ROS_ERROR("Please put at least 1 cube..."); return 1;}
  float height = 0.0254;
  float y_off = 0.1;

  geometry_msgs::Pose loc;
  loc.position.x = 0.36;
  loc.position.y = -0.44;
  loc.position.z = 0.94;
  loc.orientation.x = 0.70711;
  loc.orientation.y = 0;
  loc.orientation.z = -0.70711;
  loc.orientation.w = 0.0;

  // Const
  pr2_picknplace_msgs::PickPlaceGoal pick;
  pick.goal.request = 0;

  pr2_picknplace_msgs::PickPlaceGoal place;
  place.goal.request = 1;

  pick.goal.object_pose = loc;
  pick.goal.object_pose.position.z += ((n_cubes - 1) * height);
  place.goal.object_pose = loc;
  place.goal.object_pose.position.y -= y_off;

  for (int cube = 0; cube < n_cubes; ++cube) {
    ac.sendGoal(pick);
    // ROS_INFO_STREAM("PickHeight: " << pick.goal.object_pose.position.z);
    pick.goal.object_pose.position.z -= height;

    ac.waitForResult(ros::Duration(10.0));
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {return 1;}

    ac.sendGoal(place);
    // ROS_INFO_STREAM("PlaceHeight: " << place.goal.object_pose.position.z);
    place.goal.object_pose.position.z += height;

    ac.waitForResult(ros::Duration(10.0));
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {return 1;}
  }

  ros::shutdown();

  return 0;
}
