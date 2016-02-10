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


  std::vector<std::string> js = move_group_right_arm.getJoints();
  for (size_t i = 0; i < js.size(); ++i) {
    ROS_INFO_STREAM(js[i]);
  }

  if (add_table_) {
    ros::WallDuration(co_wait_).sleep();
    this->AddCollisionObjs();
  }

  ROS_INFO("[PICKPLACEACTION] Starting PickPlace action server.");
  as_.start();
}

PickPlaceAction::~PickPlaceAction() {
  ros::param::del(ns_);
}

void PickPlaceAction::loadParams() {
  ROS_INFO("[PICKPLACEACTION] Loading parameters.");
  if (!nh_.getParam(ns_ + "/max_planning_time", max_planning_time)) {
    ROS_WARN("[PICKPLACEACTION] WARNING: Parameters were not loaded! Using default.");
  }
  ros::param::param(ns_ + "/max_planning_time", max_planning_time, 10.0);
  ros::param::param(ns_ + "/add_table", add_table_, true);
  ros::param::param(ns_ + "/open_gripper_pos", open_gripper_pos_, 0.08);
  ros::param::param(ns_ + "/close_gripper_pos", close_gripper_pos_, 0.00);
  ROS_INFO_STREAM("Planning time: " << max_planning_time <<
                  ", Adding table: " << (add_table_ ? "True" : "False") <<
                  ", OpenGripperPos: " << open_gripper_pos_ <<
                  ", CloseGripperPos: " << close_gripper_pos_);
}

void PickPlaceAction::init() {
  co_wait_ = 1.0f;
  exec_wait_ = 0.0f;
}

void PickPlaceAction::rosSetup() {
  pub_co = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco = nh_.advertise<moveit_msgs::AttachedCollisionObject>(
              "attached_collision_object", 10);
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
  ROS_INFO_STREAM("[PICKPLACEACTION] Current " <<
                  move_group_right_arm.getCurrentPose().pose);

  gripper_client_ = new
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>(
    "r_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up
  while (!gripper_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }
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

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("[PICKPLACEACTION] %s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    // going = false;
  }

  // Call the MoveIt planning
  // AddAttachedCollBox(pick_place_goal_.object_pose);
  // Wait for ros things to initialize
  switch (pick_place_goal_.request) {
  case picknplace::REQUEST_PICK:
    PickCube(pick_place_goal_.object_pose);
    break;
  case picknplace::REQUEST_PLACE:
    PlaceCube(pick_place_goal_.object_pose);
    break;
  case picknplace::REQUEST_MOVE:
    ROS_WARN("Move Request not implemented yet.");
    break;
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
  collision_object.header.stamp = ros::Time::now();

  // The id of the object is used to identify it.
  collision_object.id = "table_top";
  // Remove any previous occurances in the world
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(
    geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.0254;
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.6;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_top_pose;
  table_top_pose.orientation.w = 1.0;
  table_top_pose.position.x =  0.8;
  table_top_pose.position.y =  -0.50;
  table_top_pose.position.z =  0.7;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_top_pose);
  collision_object.operation = collision_object.ADD;

  pub_co.publish(collision_object);
  ROS_INFO("[PICKPLACEACTION] Adding an object into the world ...");
  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  // planning_scene_interface.addCollisionObjects(collision_objects);
}

void PickPlaceAction::AddAttachedCollBox(geometry_msgs::Pose p) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_right_arm.getPlanningFrame();
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = "cube";
  // Remove any previous occurances in the world
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);

  // Create the actual object
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(
    geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0254;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.0254;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.0254;

  geometry_msgs::Pose cb_pose(p);

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cb_pose);
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  pub_co.publish(collision_object);

  ros::WallDuration(co_wait_).sleep();
  // Now define a AttachedCollisionObject
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = collision_object;
  aco.link_name = "r_wrist_roll_link";
  ROS_INFO_STREAM("Atatchable object: " << aco);
  pub_aco.publish(aco);
  ros::WallDuration(co_wait_).sleep();
}

bool PickPlaceAction::PickCube(geometry_msgs::Pose p) {
  ROS_INFO_STREAM("[PICKPLACEACTION] Starting Pick planning ...");
  moveit::planning_interface::MoveGroup::Plan pregrasp_plan;
  moveit::planning_interface::MoveGroup::Plan grasp_plan;
  moveit::planning_interface::MoveGroup::Plan postgrasp_plan;

  geometry_msgs::Pose pregrasp_pose(p);
  pregrasp_pose.position.z += 0.1;
  geometry_msgs::Pose postgrasp_pose(p);
  postgrasp_pose.position.z += 0.1;

  moveit::core::RobotState pregrasp_robot_state = RobotStateFromPose(
                                                    pregrasp_pose);
  moveit::core::RobotState grasp_robot_state = RobotStateFromPose(p);
  moveit::core::RobotState postgrasp_robot_state = RobotStateFromPose(
                                                     postgrasp_pose);

  bool success = Plan(*move_group_right_arm.getCurrentState(),
                      pregrasp_robot_state,
                      pregrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'pregrasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(pregrasp_robot_state, grasp_robot_state,
                  grasp_plan, p.orientation);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'grasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(grasp_robot_state, postgrasp_robot_state, postgrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'postgrasp': " <<
                  ((success) ? "success" : "fail"));

  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    SendGripperCommand(open_gripper_pos_);

    success = move_group_right_arm.execute(pregrasp_plan);
    ros::WallDuration(exec_wait_).sleep();

    if (success) { success &= CheckGripperFinished();}

    if (success) { success &= move_group_right_arm.execute(grasp_plan); }
    ros::WallDuration(exec_wait_).sleep();

    SendGripperCommand(close_gripper_pos_, 50.0);

    if (success) { success &= CheckGripperFinished(); }

    if (success) { success &= move_group_right_arm.execute(postgrasp_plan); }
    ROS_INFO_STREAM("[PICKPLACEACTION] MoveIt execution of plan: "
                    << ((success) ? "success" : "fail"));
  } else {
    ROS_INFO_STREAM("[PICKPLACEACTION] Failed to find a plan for pose: "
                    << pick_place_goal_.object_pose);
  }
  return success;
}

bool PickPlaceAction::PlaceCube(geometry_msgs::Pose p) {
  ROS_INFO_STREAM("[PICKPLACEACTION] Starting Place planning ...");
  moveit::planning_interface::MoveGroup::Plan preplace_plan;
  moveit::planning_interface::MoveGroup::Plan place_plan;
  moveit::planning_interface::MoveGroup::Plan postplace_plan;

  geometry_msgs::Pose preplace_pose(p);
  preplace_pose.position.z += 0.1;
  geometry_msgs::Pose postplace_pose(p);
  postplace_pose.position.z += 0.1;

  moveit::core::RobotState preplace_robot_state = RobotStateFromPose(
                                                    preplace_pose);
  moveit::core::RobotState place_robot_state = RobotStateFromPose(p);
  moveit::core::RobotState postplace_robot_state = RobotStateFromPose(
                                                     postplace_pose);

  bool success = Plan(*move_group_right_arm.getCurrentState(),
                      preplace_robot_state,
                      preplace_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'preplace': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(preplace_robot_state, place_robot_state,
                  place_plan, p.orientation);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'place': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(place_robot_state, postplace_robot_state, postplace_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'postplace': " <<
                  ((success) ? "success" : "fail"));

  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    success = move_group_right_arm.execute(preplace_plan);
    ros::WallDuration(exec_wait_).sleep();

    if (success) { success &= move_group_right_arm.execute(place_plan); }
    ros::WallDuration(exec_wait_).sleep();

    SendGripperCommand(open_gripper_pos_);

    if (success) { success &= CheckGripperFinished(); }

    if (success) { success &= move_group_right_arm.execute(postplace_plan); }
    ROS_INFO_STREAM("[PICKPLACEACTION] MoveIt execution of plan: "
                    << ((success) ? "success" : "fail"));
  } else {
    ROS_INFO_STREAM("[PICKPLACEACTION] Failed to find a plan for pose: "
                    << pick_place_goal_.object_pose);
  }
  return success;
}

bool PickPlaceAction::Plan(moveit::core::RobotState start,
                           moveit::core::RobotState end,
                           moveit::planning_interface::MoveGroup::Plan& plan,
                           geometry_msgs::Quaternion orient_constraint) {
  if (orient_constraint.x != 0.0f && orient_constraint.y != 0.0f &&
      orient_constraint.z != 0.0f && orient_constraint.w != 0.0f) {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = move_group_right_arm.getPlanningFrame();
    // Set the orientation of the final pose
    ocm.orientation = orient_constraint;

    ocm.weight = 1.0;
    //Set path constraint
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_right_arm.setPathConstraints(test_constraints);
  }

  move_group_right_arm.setStartState(start);
  move_group_right_arm.setJointValueTarget(end);
  bool success = move_group_right_arm.plan(plan);

  if (orient_constraint.x != 0.0f && orient_constraint.y != 0.0f &&
      orient_constraint.z != 0.0f && orient_constraint.w != 0.0f) {
    // Clear path constraint
    move_group_right_arm.clearPathConstraints();
  }

  return success;
}

moveit::core::RobotState PickPlaceAction::RobotStateFromPose(
  geometry_msgs::Pose p) {
  moveit::core::RobotState state(*move_group_right_arm.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group =
    state.getJointModelGroup(move_group_right_arm.getName());
  state.setFromIK(joint_model_group, p);
  return state;
}

void PickPlaceAction::SendGripperCommand(float position, float max_effort) {
  pr2_controllers_msgs::Pr2GripperCommandGoal cm;
  cm.command.position = position;
  cm.command.max_effort = max_effort;
  ROS_INFO("Sending gripper command");
  gripper_client_->sendGoal(cm);
}

bool PickPlaceAction::CheckGripperFinished() {
  gripper_client_->waitForResult(ros::Duration(max_planning_time));
  if (gripper_client_->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    ROS_WARN("[PICKPLACEACTION] The gripper failed to open.");
    return false;
  }
}
