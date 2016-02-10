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
#include <actionlib/client/simple_action_client.h>
#include <geometric_shapes/solid_primitive_dims.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

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
    // Methods
    void AddCollisionObjs();
    void AddAttachedCollBox(geometry_msgs::Pose p);
    bool PickCube(geometry_msgs::Pose p);
    bool PlaceCube(geometry_msgs::Pose p);

    bool Plan(moveit::core::RobotState start, moveit::core::RobotState end,
              moveit::planning_interface::MoveGroup::Plan& plan,
              geometry_msgs::Quaternion orient_constraint = geometry_msgs::Quaternion());
    moveit::core::RobotState RobotStateFromPose(geometry_msgs::Pose p);

    bool GripperCommand(float position, float max_effort = -1.0f);

    // Flags
    bool going;

    // Parameters
    bool add_table_;

    // Constants
    const float CLOSE_GRIPPER_POS = 0.00f;
    const float OPEN_GRIPPER_POS = 0.08f;
    float CUBE_GRIPPER_POS = 0.025f;

    // Variables
    std::string ns_;
    float wait_;
    double max_planning_time;
    pr2_picknplace_msgs::PicknPlaceGoal pick_place_goal_;
    moveit::planning_interface::MoveGroup move_group_right_arm;
    ros::Publisher pub_co;
    ros::Publisher pub_aco;
    actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>*
    gripper_client_;
};

#endif  /* PICK_PLACE_HPP */
