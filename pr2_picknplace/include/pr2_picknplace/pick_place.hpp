/**
 * @file      pick_place.hpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef PICK_PLACE_HPP
#define PICK_PLACE_HPP

// System
#include <string>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>

// MoveIt
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

class PickPlaceAction {
  protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line.
    // Otherwise strange error may occur.
    actionlib::SimpleActionServer<pr2_picknplace_msgs::PickPlaceAction> as_;
    std::string action_name_;
    pr2_picknplace_msgs::PickPlaceFeedback feedback_;
    pr2_picknplace_msgs::PickPlaceResult result_;

  public:
    PickPlaceAction(ros::NodeHandle& nh, std::string name);

    ~PickPlaceAction(void);

    void loadParams();
    void init();
    void rosSetup();

    void goalCB();

    void preemptCB();

    void executeCB();

  private:
    // Flags
    bool going;

    // Parameters
    bool add_table_;

    // Variables
    std::string ns_;
    double max_planning_time;
    pr2_picknplace_msgs::PicknPlaceGoal pick_place_goal_;
    moveit::planning_interface::MoveGroup move_group_right_arm;
    ros::Publisher pub_co;
    ros::Publisher pub_aco;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit_msgs::PlanningScene planning_scene;
    // ros::Publisher display_publisher;
    // moveit_msgs::DisplayTrajectory display_trajectory;

    // Methods
    void AddCollisionObjs();
    void AddAttachedCollBox(geometry_msgs::Pose p);
};

#endif  /* PICK_PLACE_HPP */
