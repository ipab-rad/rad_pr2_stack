/**
 * @file      pick_place.cpp
 * @brief     Provides basic pick and place API through an Action Server
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_picknplace/pick_place.hpp"
#include <tf2_eigen/tf2_eigen.h>

PickPlaceAction::PickPlaceAction(ros::NodeHandle& nh, std::string name,
                                 std::string arm) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  move_group_arm(arm),
  tfListener(tfBuffer) {
  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&PickPlaceAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PickPlaceAction::preemptCB, this));

  ns_ = ros::this_node::getNamespace() + name;

  if (arm == "right_arm") {
    wrist_roll_link = "r_wrist_roll_link";
    gripper_controller = "r_gripper_controller/gripper_action";
    gripper_tool_frame = "r_gripper_tool_frame";
    sensor_gripper_controller_grab = "r_gripper_sensor_controller/grab";
    sensor_gripper_controller_release = "r_gripper_sensor_controller/release";
  } else if (arm == "left_arm") {
    wrist_roll_link = "l_wrist_roll_link";
    gripper_controller = "l_gripper_controller/gripper_action";
    gripper_tool_frame = "l_gripper_tool_frame";
    sensor_gripper_controller_grab = "l_gripper_sensor_controller/grab";
    sensor_gripper_controller_release = "l_gripper_sensor_controller/release";
  } else {
    ROS_ERROR_STREAM("Wrong parameter 'arm'. Given: '" << arm
                     << "' was expecting 'left_arm' or 'right_arm'.");
    return;
  }

  this->loadParams();
  this->init();
  this->rosSetup();

  std::vector<std::string> js = move_group_arm.getJoints();
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
  deleteObject("cube");
  deleteObject("table_top");
  ros::param::del(ns_);
}

void PickPlaceAction::loadParams() {
  ROS_INFO("[PICKPLACEACTION] Loading parameters.");
  ROS_INFO_STREAM("NAMESPACE: " << ns_);
  if (!ros::param::get(ns_ + "/max_planning_time", max_planning_time)) {
    ROS_WARN("[PICKPLACEACTION] Parameters were not loaded! Using default.");
  }
  ros::param::param(ns_ + "/max_planning_time", max_planning_time, 10.0);
  ros::param::param(ns_ + "/add_table", add_table_, true);
  ros::param::param(ns_ + "/open_gripper_pos", open_gripper_pos_, 0.08);
  ros::param::param(ns_ + "/close_gripper_pos", close_gripper_pos_, 0.00);
  ros::param::param(ns_ + "/close_effort", close_effort_, 50.0);
  ros::param::param(ns_ + "/use_touch_pads", use_touch_pads, true);

  ROS_INFO_STREAM("Planning time: " << max_planning_time <<
                  ", Adding table: " << (add_table_ ? "True" : "False") <<
                  ", OpenGripperPos: " << open_gripper_pos_ <<
                  ", CloseGripperPos: " << close_gripper_pos_ <<
                  ", CloseEffort: " << close_effort_ <<
                  ", use_touch_pads: " << use_touch_pads);
}

void PickPlaceAction::init() {
  co_wait_ = 0.5f;
  exec_wait_ = 0.0f;

  sensor_grabbing = false;
  sensor_releasing = false;
}

void PickPlaceAction::rosSetup() {
  pub_co = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  pub_aco = nh_.advertise<moveit_msgs::AttachedCollisionObject>(
              "attached_collision_object", 10);
  // display_publisher =
  //   node_handle.advertise<moveit_msgs::DisplayTrajectory>(
  //     "/move_group/display_planned_path", 1, true);

  move_group_arm.setPlanningTime(max_planning_time);  // Seconds

  // Name of the reference frame for this robot.
  ROS_DEBUG("[PICKPLACEACTION] Reference frame: %s",
            move_group_arm.getPlanningFrame().c_str());

  // Name of the end-effector link for this group.
  ROS_DEBUG("[PICKPLACEACTION] Reference frame: %s",
            move_group_arm.getEndEffectorLink().c_str());
  // ROS_INFO_STREAM("State: " << *move_group_arm.getCurrentState());
  ROS_INFO_STREAM("[PICKPLACEACTION] Current " <<
                  move_group_arm.getCurrentPose().pose);

  if (use_touch_pads) {
    grab_client_  =
      new SensorGrabClient(sensor_gripper_controller_grab, true);
    release_client_  =
      new SensorReleaseClient(sensor_gripper_controller_release, true);
    // Wait for the servers to come up
    while (!grab_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the r/l_gripper_sensor_controller/grab action server to come up");
    }
    while (!release_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the r/l_gripper_sensor_controller/release action server to come up");
    }
  } else {
    gripper_client_ = new
    actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>(
      gripper_controller, true);
    // Wait for the gripper action server to come up
    while (!gripper_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the gripper action server to come up");
    }
  }

  deleteObject("cube");
  deleteObject("table_top");
}

void PickPlaceAction::goalCB() {
  pick_place_goal_ = as_.acceptNewGoal()->goal;
  ROS_INFO_STREAM("[PICKPLACEACTION] Received a goal for" <<
                  int(pick_place_goal_.request)  << " - " <<
                  pick_place_goal_.object_pose);
  this->executeCB();
}

void PickPlaceAction::preemptCB() {
  // Received preempt request to stop
  as_.setPreempted();
}

void PickPlaceAction::executeCB() {
  ROS_INFO("[PICKPLACEACTION] Executing goal for %s", action_name_.c_str());
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("[PICKPLACEACTION] %s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
  }

  // Call the MoveIt planning
  // AddAttachedCollBox(pick_place_goal_.object_pose);
  // Wait for ros things to initialize
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = pick_place_goal_.header.frame_id;
  ps.header.stamp = ros::Time::now();
  ps.pose = pick_place_goal_.object_pose;
  ps.pose.position.z += 0.015;  // Temporary fix until better transformation

  switch (pick_place_goal_.request) {
    case picknplace::REQUEST_PICK:
      PickCube(ps);
      break;
    case picknplace::REQUEST_PLACE:
      PlaceCube(ps);
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
  moveit_msgs::CollisionObject collision_object = deleteObject("table_top");

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(
    geometric_shapes::SolidPrimitiveDimCount
    <shape_msgs::SolidPrimitive::CYLINDER>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.0254;
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.6;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_top_pose;
  table_top_pose.orientation.w = 1.0;
  table_top_pose.position.x =  1.0;
  table_top_pose.position.y =  0.0;
  table_top_pose.position.z =  0.72;

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
  moveit_msgs::CollisionObject collision_object = deleteObject("cube");

  // Create the actual object
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(
    geometric_shapes::SolidPrimitiveDimCount
    <shape_msgs::SolidPrimitive::BOX>::value);
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
  aco.link_name = wrist_roll_link;
  ROS_INFO_STREAM("Atatchable object: " << aco);
  pub_aco.publish(aco);
  ros::WallDuration(co_wait_).sleep();
}

bool PickPlaceAction::PickCube(geometry_msgs::PoseStamped ps) {
  ROS_INFO_STREAM("[PICKPLACEACTION] Starting Pick planning ...");
  moveit::planning_interface::MoveGroup::Plan pregrasp_plan;
  moveit::planning_interface::MoveGroup::Plan grasp_plan;
  moveit::planning_interface::MoveGroup::Plan postgrasp_plan;

  std::string pr2_frame = move_group_arm.getPlanningFrame();
  pr2_frame.erase(0, 1);
  // ROS_INFO_STREAM("PR2 planning frame " << pr2_frame.c_str());

  geometry_msgs::TransformStamped transform;
  try {
    Eigen::Translation3d t0;
    t0 = Eigen::Translation3d(ps.pose.position.x,
                              ps.pose.position.y,
                              ps.pose.position.z);

    // Tabletop to OdomCombined transformation
    Eigen::Affine3d t1 =
      tf2::transformToEigen(tfBuffer.lookupTransform(pr2_frame,
                                                     ps.header.frame_id,
                                                     ros::Time(0)));
    // Rotate Gripper 90 deg in Y axis
    Eigen::Affine3d t2;
    t2 = Eigen::AngleAxisd(0.5 * M_PI,  Eigen::Vector3d::UnitY());

    // ToolFrame to WristFrame transformation
    Eigen::Affine3d t3 =
      tf2::transformToEigen(tfBuffer.lookupTransform(wrist_roll_link,
                                                     gripper_tool_frame,
                                                     ros::Time(0)));

    Eigen::Affine3d t = t1 * t0 * t2 * t0.inverse();

    ROS_INFO_STREAM("T3:\n" << t3.matrix());

    transform.transform.translation.x = t.translation().x();
    transform.transform.translation.y = t.translation().y();
    transform.transform.translation.z = t.translation().z();

    Eigen::Quaterniond q(t.rotation());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  geometry_msgs::PoseStamped pso;
  try { tf2::doTransform(ps, pso, transform); }
  catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  geometry_msgs::Pose p = pso.pose;

  ROS_DEBUG_STREAM("Input " << ps);
  ROS_DEBUG_STREAM("Output " << pso);

  geometry_msgs::Pose pregrasp_pose(p);
  pregrasp_pose.position.z += 0.1;
  geometry_msgs::Pose postgrasp_pose(p);
  postgrasp_pose.position.z += 0.1;

  ROS_DEBUG_STREAM("Pregrasp " << pregrasp_pose);
  ROS_DEBUG_STREAM("Grasp " << p);
  ROS_DEBUG_STREAM("Postgrasp " << postgrasp_pose);

  moveit::core::RobotState pregrasp_robot_state =
    RobotStateFromPose(pregrasp_pose);
  moveit::core::RobotState grasp_robot_state = RobotStateFromPose(p);
  moveit::core::RobotState postgrasp_robot_state =
    RobotStateFromPose(postgrasp_pose);

  bool success = Plan(*move_group_arm.getCurrentState(),
                      pregrasp_robot_state,
                      pregrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'pregrasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(pregrasp_robot_state, grasp_robot_state,
                  grasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'grasp': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(grasp_robot_state, postgrasp_robot_state, postgrasp_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'postgrasp': " <<
                  ((success) ? "success" : "fail"));

  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    if (use_touch_pads) {
      SensorRelease();
    } else {
      SendGripperCommand(open_gripper_pos_);
    }

    success = move_group_arm.execute(pregrasp_plan);
    ros::WallDuration(exec_wait_).sleep();

    if (success) { success &= CheckGripperFinished();}

    if (success) { success &= move_group_arm.execute(grasp_plan); }
    ros::WallDuration(exec_wait_).sleep();

    if (use_touch_pads) {
      SensorGrab();
    } else {
      SendGripperCommand(close_gripper_pos_, close_effort_);
    }

    if (success) { success &= CheckGripperFinished(); }

    ros::WallDuration(0.1).sleep();  // Gripper delay for better grippage

    if (success) { success &= move_group_arm.execute(postgrasp_plan); }
    ROS_INFO_STREAM("[PICKPLACEACTION] MoveIt execution of plan: "
                    << ((success) ? "success" : "fail"));
  } else {
    ROS_INFO_STREAM("[PICKPLACEACTION] Failed to find a plan for pose: "
                    << pick_place_goal_.object_pose);
  }
  return success;
}

bool PickPlaceAction::PlaceCube(geometry_msgs::PoseStamped ps) {
  ROS_INFO_STREAM("[PICKPLACEACTION] Starting Place planning ...");
  moveit::planning_interface::MoveGroup::Plan preplace_plan;
  moveit::planning_interface::MoveGroup::Plan place_plan;
  moveit::planning_interface::MoveGroup::Plan postplace_plan;

  std::string pr2_frame = move_group_arm.getPlanningFrame();
  pr2_frame.erase(0, 1);

  geometry_msgs::TransformStamped transform;
  try {
    Eigen::Translation3d t0;
    t0 = Eigen::Translation3d(ps.pose.position.x,
                              ps.pose.position.y,
                              ps.pose.position.z);

    // Tabletop to OdomCombined transformation
    Eigen::Affine3d t1 =
      tf2::transformToEigen(tfBuffer.lookupTransform(pr2_frame,
                                                     ps.header.frame_id,
                                                     ros::Time(0)));
    // Rotate Gripper 90 deg in Y axis
    Eigen::Affine3d t2;
    t2 = Eigen::AngleAxisd(0.5 * M_PI,  Eigen::Vector3d::UnitY());

    // ToolFrame to WristFrame transformation
    Eigen::Affine3d t3 =
      tf2::transformToEigen(tfBuffer.lookupTransform(wrist_roll_link,
                                                     gripper_tool_frame,
                                                     ros::Time(0)));

    Eigen::Affine3d t = t1 * t0 * t2 * t0.inverse();

    ROS_INFO_STREAM("T3:\n" << t3.matrix());

    transform.transform.translation.x = t.translation().x();
    transform.transform.translation.y = t.translation().y();
    transform.transform.translation.z = t.translation().z();

    Eigen::Quaterniond q(t.rotation());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  geometry_msgs::PoseStamped pso;
  try { tf2::doTransform(ps, pso, transform); }
  catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  geometry_msgs::Pose p = pso.pose;

  ROS_DEBUG_STREAM("Input " << ps);
  ROS_DEBUG_STREAM("Output " << pso);

  geometry_msgs::Pose preplace_pose(p);
  preplace_pose.position.z += 0.1;
  geometry_msgs::Pose postplace_pose(p);
  postplace_pose.position.z += 0.1;

  ROS_DEBUG_STREAM("Preplace " << preplace_pose);
  ROS_DEBUG_STREAM("Place " << p);
  ROS_DEBUG_STREAM("Postplace " << postplace_pose);

  moveit::core::RobotState preplace_robot_state = RobotStateFromPose(
                                                    preplace_pose);
  moveit::core::RobotState place_robot_state = RobotStateFromPose(p);
  moveit::core::RobotState postplace_robot_state = RobotStateFromPose(
                                                     postplace_pose);

  bool success = Plan(*move_group_arm.getCurrentState(),
                      preplace_robot_state,
                      preplace_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'preplace': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(preplace_robot_state, place_robot_state, place_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'place': " <<
                  ((success) ? "success" : "fail"));
  success &= Plan(place_robot_state, postplace_robot_state, postplace_plan);
  ROS_INFO_STREAM("[PICKPLACEACTION] Planning 'postplace': " <<
                  ((success) ? "success" : "fail"));

  if (success) {
    ROS_INFO_STREAM("[PICKPLACEACTION] Executing on the robot ...");
    success = move_group_arm.execute(preplace_plan);
    ros::WallDuration(exec_wait_).sleep();

    if (success) { success &= move_group_arm.execute(place_plan); }
    ros::WallDuration(exec_wait_).sleep();

    if (use_touch_pads) {
      SensorRelease();
    } else {
      SendGripperCommand(open_gripper_pos_);
    }

    if (success) { success &= CheckGripperFinished(); }

    if (success) { success &= move_group_arm.execute(postplace_plan); }
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
    ocm.link_name = wrist_roll_link;
    ocm.header.frame_id = move_group_arm.getPlanningFrame();
    // Set the orientation of the final pose
    ocm.orientation = orient_constraint;

    ocm.weight = 1.0;
    // Set path constraint
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_arm.setPathConstraints(test_constraints);
  }

  move_group_arm.setStartState(start);
  move_group_arm.setJointValueTarget(end);
  bool success = move_group_arm.plan(plan);

  if (orient_constraint.x != 0.0f && orient_constraint.y != 0.0f &&
      orient_constraint.z != 0.0f && orient_constraint.w != 0.0f) {
    // Clear path constraint
    move_group_arm.clearPathConstraints();
  }

  return success;
}

moveit::core::RobotState PickPlaceAction::RobotStateFromPose(
  geometry_msgs::Pose p) {
  moveit::core::RobotState state(*move_group_arm.getCurrentState());
  const robot_state::JointModelGroup* joint_model_group =
    state.getJointModelGroup(move_group_arm.getName());
  state.setFromIK(joint_model_group, p, gripper_tool_frame);
  return state;
}

void PickPlaceAction::SendGripperCommand(float position, float max_effort) {
  if (use_touch_pads) {
    ROS_ERROR("I am sending gripper commands to standard gripper.");
  }
  pr2_controllers_msgs::Pr2GripperCommandGoal cm;
  cm.command.position = position;
  cm.command.max_effort = max_effort;
  ROS_INFO("Sending standard gripper command");
  gripper_client_->sendGoal(cm);
}

bool PickPlaceAction::CheckGripperFinished() {
  if (use_touch_pads) {
    // Use touch sensing fingers
    if (sensor_grabbing) {
      sensor_grabbing = false;
      grab_client_->waitForResult(ros::Duration(max_planning_time));
      if (grab_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successfully completed Grab");
        return true;
      } else {
        ROS_INFO("Grab Failed");
        return false;
      }
    } else if (sensor_releasing) {
      sensor_releasing = false;
      release_client_->waitForResult(ros::Duration(max_planning_time));
      if (release_client_->getState() ==
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Release Success");
        return true;
      } else {
        ROS_INFO("Place Failure");
        return false;
      }
    } else {
      ROS_WARN_STREAM("Wrong use_touch_pads when CheckGripperFinished." <<
                      "Both sensor_grabbing: " << sensor_grabbing <<
                      " and sensor_releasing: " << sensor_releasing);
      return false;
    }
  } else {
    // Use standard gripper
    gripper_client_->waitForResult(ros::Duration(max_planning_time));
    if (gripper_client_->getState() ==
        actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else {
      ROS_WARN("[PICKPLACEACTION] The gripper failed to open.");
      return false;
    }
  }
}

bool PickPlaceAction::SensorGrab() {
  pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip;
  grip.command.hardness_gain = 0.03;

  ROS_INFO("Sending grab goal");
  grab_client_->sendGoal(grip);
  sensor_grabbing = true;
  return true;
}

bool PickPlaceAction::SensorRelease() {
  pr2_gripper_sensor_msgs::PR2GripperReleaseGoal place;
  // set the robot to release on a figner-side impact, fingerpad slip, or acceleration impact with hand/arm
  place.command.event.trigger_conditions =
    place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
  // set the acceleration impact to trigger on to 5 m/s^2
  // If set to 0, will release instantaniously
  place.command.event.acceleration_trigger_magnitude = 0.0;
  // set our slip-gain to release on to .005
  place.command.event.slip_trigger_magnitude = .01;

  ROS_INFO("Waiting for object placement contact...");
  release_client_->sendGoal(place);
  sensor_releasing = true;
  return true;
}

moveit_msgs::CollisionObject PickPlaceAction::deleteObject(
  std::string object_id) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = object_id;
  // Remove any previous occurrences in the world
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);
  // ros::WallDuration(co_wait_).sleep();
  return collision_object;
}
