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

  ns_ = ros::this_node::getNamespace() + name;

  this->loadParams();
  this->init();
  this->rosSetup();

  // ros::Duration(1.0).sleep();

  ROS_INFO("[PICKPLACEACTION] Starting PickPlace action server.");
  as_.start();
}

PickPlaceAction::~PickPlaceAction() {
  ros::param::del("pick_place");
}

void PickPlaceAction::loadParams() {
  ROS_INFO("[PICKPLACEACTION] Loading parameters.");
  if (!nh_.getParam(ns_ + "/max_planning_time", max_planning_time)) {
    ROS_WARN("[PICKPLACEACTION] WARNING: Parameters were not loaded! Using default.");
  }
  ros::param::param(ns_ + "/max_planning_time", max_planning_time, 10.0);
  ros::param::param(ns_ + "/add_table", add_table_, true);
  ROS_INFO_STREAM(max_planning_time << add_table_);
}

void PickPlaceAction::init() {
}

void PickPlaceAction::rosSetup() {
  // display_publisher =
  //   node_handle.advertise<moveit_msgs::DisplayTrajectory>(
  //     "/move_group/display_planned_path", 1, true);

  move_group_right_arm.setPlanningTime(max_planning_time); //seconds

  // Name of the reference frame for this robot.
  ROS_DEBUG("[PICKPLACEACTION] Reference frame: %s",
            move_group_right_arm.getPlanningFrame().c_str());

  // Name of the end-effector link for this group.
  ROS_DEBUG("[PICKPLACEACTION] Reference frame: %s",
            move_group_right_arm.getEndEffectorLink().c_str());
  // ROS_INFO_STREAM("State: " << *move_group_right_arm.getCurrentState());
  ROS_DEBUG_STREAM("[PICKPLACEACTION] Current pose" <<
                   move_group_right_arm.getCurrentPose().pose);
}

void PickPlaceAction::goalCB() {
  pick_place_goal_ = as_.acceptNewGoal()->goal;
  ROS_INFO_STREAM("[PICKPLACEACTION] Received a goal for" <<
                  pick_place_goal_.request  << " - " <<
                  pick_place_goal_.object_pose);
  this->executeCB();
}

void PickPlaceAction::preemptCB() {
  // Received preempt request to stop
  // going = false;
  as_.setPreempted();
}

void PickPlaceAction::executeCB() {
  ROS_INFO("[PICKPLACEACTION] Executing goal for %s", action_name_.c_str());
  // bool going = true;
  bool success = true;
  if (add_table_) {this->AddCollisionObjs();}

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("[PICKPLACEACTION] %s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    // going = false;
  }

  // Call the MoveIt planning
  ROS_INFO_STREAM("[PICKPLACEACTION] Starting MoveIt planning ...");
  move_group_right_arm.setPoseTarget(pick_place_goal_.object_pose,
                                     "r_wrist_roll_link");
  moveit::planning_interface::MoveGroup::Plan obj_pose_plan;
  success = move_group_right_arm.plan(obj_pose_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning finished: " <<
                  ((success) ? "success" : "fail"));

  // display_trajectory.trajectory_start = obj_pose_plan.start_state_;
  // display_trajectory.trajectory.push_back(obj_pose_plan.trajectory_);
  // display_publisher.publish(display_trajectory);
  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    success = move_group_right_arm.execute(obj_pose_plan);
    ROS_INFO_STREAM("[PICKPLACEACTION] Return success of MoveIt execution of plan: "
                    << ((success) ? "success" : "fail"));
  } else {
    ROS_INFO_STREAM("[PICKPLACEACTION] Failed to find a plan for pose: "
                    << pick_place_goal_.object_pose);
  }

  if (success) {
    result_.success = true;
    ROS_INFO("[PICKPLACEACTION] %s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("[PICKPLACEACTION] %s: Failed!", action_name_.c_str());
    as_.setSucceeded(result_);
  }

}


void PickPlaceAction::AddCollisionObjs() {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_right_arm.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "table_top";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.0254;  // Height of cylinder
  primitive.dimensions[1] = 0.6;    // Radius

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose table_top_pose;
  table_top_pose.orientation.w = 1.0;
  table_top_pose.position.x =  1.0;
  table_top_pose.position.y =  0.0;
  table_top_pose.position.z =  0.7;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_top_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add an object into the world ...");
  planning_scene_interface.addCollisionObjects(collision_objects);
}
