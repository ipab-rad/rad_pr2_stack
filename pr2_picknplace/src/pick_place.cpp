/**
 * @file      pick_place.cpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_picknplace/pick_place.hpp"

PickPlaceAction::PickPlaceAction(ros::NodeHandle& nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  move_group_right_arm("right_arm") {
  // Register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PickPlaceAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PickPlaceAction::preemptCB, this));

  ROS_INFO("Loading parameters.");
  this->loadParams();
  this->init();
  this->rosSetup();

  ROS_INFO("Starting PickPlace action server.");
  as_.start();
}

PickPlaceAction::~PickPlaceAction() {
  ros::param::del("pick_place");
}

void PickPlaceAction::loadParams() {
}

void PickPlaceAction::init() {
}

void PickPlaceAction::rosSetup() {
  // display_publisher =
  //   node_handle.advertise<moveit_msgs::DisplayTrajectory>(
  //     "/move_group/display_planned_path", 1, true);
}

void PickPlaceAction::goalCB() {
  pick_place_goal_ = as_.acceptNewGoal()->goal;
  ROS_INFO_STREAM("Received a goal for" << pick_place_goal_.request  <<
                  " - " << pick_place_goal_.object_pose);
  this->executeCB();
}

void PickPlaceAction::preemptCB() {
  // Received preempt request to stop
  // going = false;
  as_.setPreempted();
}

void PickPlaceAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  // bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    // going = false;
  }

  // Call the MoveIt magic
  move_group_right_arm.setPoseTarget(pick_place_goal_.object_pose);
  moveit::planning_interface::MoveGroup::Plan obj_pose_plan;
  success = move_group_right_arm.plan(obj_pose_plan);
  // display_trajectory.trajectory_start = obj_pose_plan.start_state_;
  // display_trajectory.trajectory.push_back(obj_pose_plan.trajectory_);
  // display_publisher.publish(display_trajectory);
  if (success) {
    success = move_group_right_arm.execute(obj_pose_plan);
    ROS_DEBUG_STREAM("Return success of MoveIt execution of plan: " << success);
  } else {
    ROS_DEBUG_STREAM("Failed to find a plan for pose: "
                     << pick_place_goal_.object_pose);
  }

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setSucceeded(result_);
  }

}
