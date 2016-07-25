/**
 * @file      pick_place.hpp
 * @brief     Provides basic pick and place API through an Action Server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef PICK_PLACE_HPP
#define PICK_PLACE_HPP

// System
#include <string>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

// Sensor grasping
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>

// The sensor msgs action aclient typedef
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction>
SensorGrabClient;
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction>
SensorReleaseClient;

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
    PickPlaceAction(ros::NodeHandle& nh, std::string name, std::string arm);

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
    bool PickCube(geometry_msgs::PoseStamped ps);
    bool PlaceCube(geometry_msgs::PoseStamped ps);
    bool MoveTo(geometry_msgs::PoseStamped ps);
    bool Push(geometry_msgs::PoseStamped ps);

    bool Plan(moveit::core::RobotState start, moveit::core::RobotState end,
              moveit::planning_interface::MoveGroup::Plan& plan,
              geometry_msgs::Quaternion orient_constraint =
                  geometry_msgs::Quaternion());
    moveit::core::RobotState CreateEmptyRobotState();
    bool RobotStateFromPose(const geometry_msgs::Pose p,
                            moveit::core::RobotState& robot_sate);

    bool ConvertPoseToGrabPose(const geometry_msgs::PoseStamped& ps,
                               geometry_msgs::Pose& ps_out);

    void SendGripperCommand(float position, float max_effort = -1.0f);
    bool CheckGripperFinished();
    moveit_msgs::CollisionObject deleteObject(std::string object_id);
    bool SensorGrab();
    bool SensorRelease();
    void gripperSlipCallback(
        const pr2_gripper_sensor_msgs::PR2GripperSlipServoActionFeedback::Ptr msg);

    // Flags
    bool sensor_grabbing;
    bool sensor_releasing;

    // Parameters
    std::string wrist_roll_link;
    std::string gripper_controller;
    std::string gripper_tool_frame;
    bool add_table_;
    double max_planning_time_;
    int num_planning_attempts_;
    double open_gripper_pos_;
    double close_gripper_pos_;
    double close_effort_;
    bool use_touch_pads;
    std::string sensor_gripper_controller_grab;
    std::string sensor_gripper_controller_release;
    std::string sensor_gripper_controller_param;
    double hardness_gain;
    double close_speed;
    double fingertip_force_limit;
    double deformation_limit;
    double force_lightest;
    double position_open;


    // Variables
    std::string ns_;
    float co_wait_;
    float exec_wait_;
    pr2_picknplace_msgs::PicknPlaceGoal pick_place_goal_;
    moveit::planning_interface::MoveGroup move_group_arm;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    bool is_gripper_empty;

    ros::Publisher pub_co;
    ros::Publisher pub_aco;
    actionlib::SimpleActionClient
    <pr2_controllers_msgs::Pr2GripperCommandAction>* gripper_client_;
    SensorGrabClient* grab_client_;
    SensorReleaseClient* release_client_;
    ros::ServiceClient sensor_update_param_service;
    ros::Subscriber gripper_slip_sub;
};

#endif  /* PICK_PLACE_HPP */
