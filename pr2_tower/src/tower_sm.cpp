/**
 * @file      tower_sm.cpp
 * @brief     PR2 Tower Building State Machine
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-09-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <pr2_tower/tower_sm.hpp>

TowerSM::TowerSM(ros::NodeHandle& nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_right_("/pr2_picknplace_right/pick_place", true),
  ac_left_("/pr2_picknplace_left/pick_place", true) {
  // Register the goal and feedback callbacks
  as_.registerGoalCallback(boost::bind(&TowerSM::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&TowerSM::preemptCB, this));

  this->loadParams();
  this->init();
  this->rosSetup();

  ROS_INFO("[PR2_TOWER_SM] Waiting for left and right PickPlaceAction servers...");
  // actionlib::SimpleActionClient<pr2_picknplace_msgs::PickPlaceAction>
  // ac_("/pr2_picknplace/pick_place", true);
  ac_right_.waitForServer();
  ac_left_.waitForServer();
  ROS_INFO("[PR2_TOWER_SM] PickPlace action client Ready!");

  ROS_INFO("[PICKPLACEACTION] Starting TowerSM action server...");
  as_.start();
  ROS_INFO("[PR2_TOWER_SM] TowerSM action server Ready!");
}

TowerSM::~TowerSM() {
  ros::param::del("pr2_tower");
}

void TowerSM::loadParams() {
  ROS_INFO("[PR2_TOWER_SM] Loading parameters.");
  nh_.getParam("num_blocks", num_blocks_);
  ROS_DEBUG_STREAM("[PR2_TOWER_SM] NumBlocks: " << num_blocks_);
  nh_.getParam("block_height", block_height_);

  // Get desired final pose of tower
  nh_.getParam("tower/position/x", tower_pose_.position.x);
  nh_.getParam("tower/position/y", tower_pose_.position.y);
  nh_.getParam("tower/position/z", tower_pose_.position.z);
  nh_.getParam("tower/orientation/x", tower_pose_.orientation.x);
  nh_.getParam("tower/orientation/y", tower_pose_.orientation.y);
  nh_.getParam("tower/orientation/z", tower_pose_.orientation.z);
  nh_.getParam("tower/orientation/w", tower_pose_.orientation.w);

  // Check if tower should be dissassembled after construction
  nh_.getParam("dissassemble", dissassemble_);
  bool preallocation;
  nh_.getParam("preallocation", preallocation);
  if (preallocation) {   // Check the pose of blocks if they have been provided
    for (int i = 0; i < num_blocks_; ++i) {
      geometry_msgs::Pose block;
      std::string id =  std::to_string(i);
      if (nh_.hasParam("block_" + id)) {
        nh_.getParam("block_" + id + "/position/x", block.position.x);
        nh_.getParam("block_" + id + "/position/y", block.position.y);
        nh_.getParam("block_" + id + "/position/z", block.position.z);
        nh_.getParam("block_" + id + "/orientation/x", block.orientation.x);
        nh_.getParam("block_" + id + "/orientation/y", block.orientation.y);
        nh_.getParam("block_" + id + "/orientation/z", block.orientation.z);
        nh_.getParam("block_" + id + "/orientation/w", block.orientation.w);
        block_poses_.push_back(block);
      } else {
        ROS_ERROR("Incorrect number of blocks!");
        ros::shutdown();
      }
    }
  }
}

void TowerSM::init() {
  ac_timeout_ = 120.0;
  hold_tower_ = false;
  pick_.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PICK_REQUEST;
  pick_.goal.header.frame_id = "tabletop";

  place_.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PLACE_REQUEST;
  place_.goal.header.frame_id = "tabletop";

  // Move arms to safe locations
  movetoright_.goal.request =
    pr2_picknplace_msgs::PicknPlaceGoal::MOVETO_REQUEST;
  movetoright_.goal.header.frame_id = "tabletop";
  movetoright_.goal.object_pose.position.x = 0.1;
  movetoright_.goal.object_pose.position.y = -0.55;
  movetoright_.goal.object_pose.position.z = 0.1;
  movetoright_.goal.object_pose.orientation.x = 0.0;
  movetoright_.goal.object_pose.orientation.y = 0.0;
  movetoright_.goal.object_pose.orientation.z = 0.0;
  movetoright_.goal.object_pose.orientation.w = 1.0;
  movetoleft_.goal = movetoright_.goal;
  movetoleft_.goal.object_pose.position.y *= -1;  // Reverse for left arm

  // Hold bottom of tower while building
  hold_.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::HOLD_REQUEST;
  hold_.goal.header.frame_id = "tabletop";
  hold_.goal.object_pose.position.x = 0.1;
  hold_.goal.object_pose.position.y = 0.0;
  hold_.goal.object_pose.position.z = 0.0;
  hold_.goal.object_pose.orientation.x = -0.612;
  hold_.goal.object_pose.orientation.y = 0.354;
  hold_.goal.object_pose.orientation.z = -0.354;
  hold_.goal.object_pose.orientation.w = 0.612;

  release_.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::RELEASE_REQUEST;
  release_.goal.header.frame_id = "tabletop";
  release_.goal.object_pose = hold_.goal.object_pose;
}

void TowerSM::rosSetup() {
}

void TowerSM::goalCB() {
  tower_sm_goal_.build = as_.acceptNewGoal()->build;
  ROS_INFO_STREAM("[PR2_TOWER_SM] Received a goal!");
  if (tower_sm_goal_.build) {this->executeCB();}
}

void TowerSM::preemptCB() {
  // Received preempt request to stop
  as_.setPreempted();
}

void TowerSM::executeCB() {
  ROS_INFO("[PR2_TOWER_SM] Executing goal for %s", action_name_.c_str());
  result_.success = true;

  prepareArms();

  result_.success = buildTower();

  if (result_.success && dissassemble_) {dissassembleTower();}

  prepareArms();

  if (result_.success) {
    ROS_INFO("[PICKPLACEACTION] %s: Succeeded!", action_name_.c_str());
  } else {
    ROS_INFO("[PICKPLACEACTION] %s: Failed!", action_name_.c_str());
  }
  as_.setSucceeded(result_);
}

bool TowerSM::prepareArms() {
  if (checkOK()) {
    ROS_INFO("Preparing...");
    ac_right_.sendGoal(movetoright_);
    ac_left_.sendGoal(movetoleft_);
    ac_right_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_right_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false;}
    ac_left_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false;}
  }
  return result_.success;
}

bool TowerSM::buildTower() {
  ROS_INFO("Building!");
  for (int i = 0; i < num_blocks_; ++i) {
    // Pick Block up
    ROS_INFO_STREAM("Pick Block " << i);
    if (!checkOK()) {break;}
    pick_.goal.object_pose = block_poses_[i];
    pick_.goal.object_pose.orientation.x = 0.707;
    pick_.goal.object_pose.orientation.w = 0.707;
    ac_right_.sendGoal(pick_);
    ROS_DEBUG_STREAM("PickHeight: " << pick_.goal.object_pose.position.z);
    ac_right_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_right_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false; break;}

    // Place Block down
    ROS_INFO_STREAM("Place Block " << i);
    if (!checkOK()) {break;}
    place_.goal.object_pose = tower_pose_;
    place_.goal.object_pose.position.z =
      tower_pose_.position.z + (block_height_ * i); // Add block height
    if (i <= 1) { // Avoid collision with left hand
      place_.goal.object_pose.orientation.x = 0.707;
      place_.goal.object_pose.orientation.w = 0.707;
    }
    ac_right_.sendGoal(place_);
    ac_right_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_right_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false; break;}

    // Hold first block with left hand
    if (i == 0 && hold_tower_) {
      ac_left_.sendGoal(hold_);
      ac_left_.waitForResult(ros::Duration(ac_timeout_));
      if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {result_.success = false; break;}
    }
  }
  if (hold_tower_) {releaseTower();} //  Let go of tower when completed

  return result_.success;
}

bool TowerSM::dissassembleTower() {
  ROS_INFO("Dissassembling!");
  for (int i = (num_blocks_ - 1); i >= 0 ; --i) {
    if (!checkOK()) {break;}
    // Hold lower block for dissassembly
    release_.goal.object_pose.position.z = block_height_ * i;
    ac_left_.sendGoal(release_);
    ac_left_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false; break;}
    if (i > 0) {
      hold_.goal.object_pose.position.z = block_height_ * (i - 1);
      ac_left_.sendGoal(hold_);
      ac_left_.waitForResult(ros::Duration(ac_timeout_));
      if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {result_.success = false; break;}
    } else {  // Except the very last block
      ac_left_.sendGoal(movetoleft_);
      ac_left_.waitForResult(ros::Duration(ac_timeout_));
      if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {result_.success = false; break;}
    }
    // Pick Block up
    if (!checkOK()) {break;}
    pick_.goal.object_pose = tower_pose_;
    pick_.goal.object_pose.position.z =
      tower_pose_.position.z + (block_height_ * i); // Add block height
    pick_.goal.object_pose.orientation.x = 0.707;
    pick_.goal.object_pose.orientation.w = 0.707;
    ac_right_.sendGoal(pick_);
    ROS_DEBUG_STREAM("PickHeight: " << pick_.goal.object_pose.position.z);
    ac_right_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_right_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false; break;}

    // Place Block down
    if (!checkOK()) {break;}
    place_.goal.object_pose = block_poses_[i];
    place_.goal.object_pose.position.z -= 0.01; // Place firmly on table
    place_.goal.object_pose.orientation.x = 0.707;
    place_.goal.object_pose.orientation.w = 0.707;
    ac_right_.sendGoal(place_);
    ac_right_.waitForResult(ros::Duration(ac_timeout_));
    if (ac_right_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {result_.success = false; break;}
  }

  return result_.success;
}

bool TowerSM::releaseTower() {
  ac_left_.sendGoal(release_);
  ac_left_.waitForResult(ros::Duration(ac_timeout_));
  if (ac_left_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {result_.success = false;}

  return result_.success;
}

bool TowerSM::checkOK() {
  // Check if actions server was preempted or if something went wrong
  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_WARN("[PR2_TOWER_SM] %s: Preempted", action_name_.c_str());
    as_.setPreempted();
    result_.success = false;
  }
  return result_.success;
}
