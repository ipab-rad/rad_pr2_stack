/**
 * @file      pick_place.cpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_picknplace/pick_place.hpp"
#include <geometric_shapes/solid_primitive_dims.h>

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
    ros::WallDuration(1.0).sleep();
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
  ROS_INFO_STREAM("Planning time: " << max_planning_time <<
                  "\nAdding table: " << add_table_);
}

void PickPlaceAction::init() {
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
  ROS_INFO_STREAM("[PICKPLACEACTION] Current pose" <<
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

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("[PICKPLACEACTION] %s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    // going = false;
  }

  // Call the MoveIt planning
  // AddAttachedCollBox(pick_place_goal_.object_pose);
  // Wait for ros things to initialize
  // ros::WallDuration(1.0).sleep();
  // PickCube(pick_place_goal_.object_pose);

  ROS_INFO_STREAM("[PICKPLACEACTION] Starting MoveIt planning ...");
  moveit::planning_interface::MoveGroup::Plan pregrasp_plan;
  moveit::planning_interface::MoveGroup::Plan grasp_plan;
  moveit::planning_interface::MoveGroup::Plan postgrasp_plan;

  // display_trajectory.trajectory_start = obj_pose_plan.start_state_;
  // display_trajectory.trajectory.push_back(obj_pose_plan.trajectory_);
  // display_publisher.publish(display_trajectory);

  geometry_msgs::Pose pregrasp_pose(pick_place_goal_.object_pose);
  pregrasp_pose.position.z += 0.1;
  geometry_msgs::Pose postgrasp_pose(pick_place_goal_.object_pose);
  postgrasp_pose.position.z += 0.1;

  moveit::core::RobotState pregrasp_robot_state = RobotStateFromPose(
                                                    pregrasp_pose);
  moveit::core::RobotState postgrasp_robot_state = RobotStateFromPose(
                                                     postgrasp_pose);

  success = Plan(*move_group_right_arm.getCurrentState(),
                 pregrasp_robot_state,
                 pregrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'pregrasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(pregrasp_robot_state,
                  RobotStateFromPose(pick_place_goal_.object_pose),
                  grasp_plan,
                  pick_place_goal_.object_pose.orientation);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'grasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(RobotStateFromPose(pick_place_goal_.object_pose),
                  postgrasp_robot_state,
                  postgrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'postgrasp': " <<
                  ((success) ? "success" : "fail"));

  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    success = move_group_right_arm.execute(pregrasp_plan);
    ros::WallDuration(1.0).sleep();
    if (success) { success &= move_group_right_arm.execute(grasp_plan); }
    ros::WallDuration(1.0).sleep();
    if (success) { success &= move_group_right_arm.execute(postgrasp_plan); }
    ROS_INFO_STREAM("[PICKPLACEACTION] MoveIt execution of plan: "
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
  table_top_pose.position.x =  0.5;
  table_top_pose.position.y =  0.0;
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

  ros::WallDuration(1.0).sleep();
  // Now define a AttachedCollisionObject
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = collision_object;
  aco.link_name = "r_wrist_roll_link";
  ROS_INFO_STREAM("Atatchable object: " << aco);
  pub_aco.publish(aco);
  ros::WallDuration(1.0).sleep();
}

bool PickPlaceAction::PickCube(geometry_msgs::Pose p) {
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = move_group_right_arm.getPlanningFrame();
  ps.pose = p;
  moveit_msgs::Grasp g;
  g.grasp_pose = ps;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id =
    move_group_right_arm.getPlanningFrame();
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint"); //r_gripper_joint
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 1;

  g.grasp_posture.joint_names.resize(1, "r_gripper_joint"); //r_gripper_joint
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  grasps.push_back(g);
  move_group_right_arm.setSupportSurfaceName("table_top");
  return move_group_right_arm.pick("cube", grasps);
}

bool PickPlaceAction::PlaceCube(geometry_msgs::Pose p) {
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "base_footprint";
  ps.pose = p;
  moveit_msgs::PlaceLocation g;
  g.place_pose = ps;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id =
    move_group_right_arm.getPlanningFrame();
  g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  g.post_place_posture.joint_names.resize(1, "r_gripper_joint");
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(1);
  g.post_place_posture.points[0].positions[0] = 1;

  loc.push_back(g);
  move_group_right_arm.setSupportSurfaceName("table_top");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint& ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = ps.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  move_group_right_arm.setPathConstraints(constr);
  move_group_right_arm.setPlannerId("RRTConnectkConfigDefault");

  return move_group_right_arm.place("cube", loc);
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
