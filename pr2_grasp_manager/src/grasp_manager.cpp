/**
 * @file      grasp_manager.cpp
 * @brief     Provides grasping pipeline using haf_grasping and MoveIt
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-06-08
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <pr2_grasp_manager/grasp_manager.hpp>

#include <cmath>

GraspManager::GraspManager(ros::NodeHandle& nh) :
  nh_(nh),
  haf_ac_(nh_, "/calc_grasppoints_svm_action_server", true),
  pickplace_ac_(nh_, "/pr2_picknplace/pick_place", true),
  tfListener(tfBuffer) {
  ROS_INFO("[GRASPMANAGER] Starting Grasp Manager.");
  ns_ = ros::this_node::getNamespace();

  this->loadParams();
  this->init();
  this->rosSetup();

  ROS_INFO("Waiting for Haf_Grasping action server to start.");
  haf_ac_.waitForServer();
  ROS_INFO("Haf_Grasping action server started!");
  ROS_INFO("Waiting for PicknPlace action server to start.");
  pickplace_ac_.waitForServer();
  ROS_INFO("PicknPlace action server started!");
  request_pc_.call(empty_); // Request at least once!
}

GraspManager::~GraspManager() {
  ros::param::del(ns_);
}

void GraspManager::loadParams() {
}

void GraspManager::init() {
  pc_ready_ = false;
  ready_to_grasp_ = true;
  ready_to_pick_ = false;
  ready_to_place_ = false;
  eval_thresh = 0;
  execute_motion_ = false; // Whether to actually move or not

  // Haf Grasping Goal request
  goal_.header.frame_id = "base_link";
  grasp_req_.goal_frame_id = "";
  geometry_msgs::Point grasp_center;
  grasp_center.x = 0.6;
  grasp_center.y = 0.0;
  grasp_center.z = 0.7;
  grasp_req_.grasp_area_center = grasp_center;
  grasp_req_.grasp_area_length_x = 32;
  grasp_req_.grasp_area_length_y = 32;

  grasp_req_.max_calculation_time = ros::Duration(40.0);
  grasp_req_.show_only_best_grasp = true;
  grasp_req_.threshold_grasp_evaluation = 0;

  geometry_msgs::Vector3 appr_vect;
  appr_vect.x = 0.0;
  appr_vect.y = 0.0;
  appr_vect.z = 1.0;
  grasp_req_.approach_vector = appr_vect;
  grasp_req_.gripper_opening_width = 1;

  place_pose_.position.x = 0.6;
  place_pose_.position.y = -0.5;
  place_pose_.position.z = 0.8;
  place_pose_.orientation.x = 0;
  place_pose_.orientation.y = 0;
  place_pose_.orientation.z = 0;
  place_pose_.orientation.w = 1;
}

void GraspManager::rosSetup() {
  // point_cloud_sub_ = nh_.subscribe("/plane_segmentator/extracted_outliers",
  point_cloud_sub_ = nh_.subscribe("/plane_segmentator/segmented_objects_above",
                                   10, &GraspManager::pcCB, this);
  request_pc_ = nh_.serviceClient<std_srvs::Empty>
                ("/plane_segmentator/request_pointcloud", true);
}

void GraspManager::pcCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO("Got PointCloud!");
  grasp_req_.input_pc = *msg;
  // goal_.header.stamp = msg->header.stamp;
  std_msgs::Header now;
  now.stamp = ros::Time::now();
  goal_.header.stamp = now.stamp;
  grasp_req_.input_pc.header.stamp = now.stamp;
  if (ready_to_grasp_) {
    pc_ready_ = true;
  }
}

void GraspManager::getGrasp() {
  if (!pc_ready_) {
    ROS_INFO_THROTTLE(1, "Waiting for PC...");
  } else if (!ready_to_grasp_) {
    ROS_INFO_THROTTLE(1, "Grasp not ready...");
  } else {
    pc_ready_ = false;
    ready_to_grasp_ = false;
    ROS_INFO("Sending Goal...");
    goal_.goal.graspinput = grasp_req_;
    haf_ac_.sendGoal(goal_.goal);
    ROS_INFO("Goal sent!");

    bool finished_before_timeout = haf_ac_.waitForResult(ros::Duration(50.0));
    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = haf_ac_.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
      grasp_res_ = haf_ac_.getResult()->graspOutput;
      std::cout << grasp_res_;
      if (grasp_res_.eval > eval_thresh) { ready_to_pick_ = true; }
      else {request_pc_.call(empty_); ready_to_grasp_ = true; }
    } else {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
}

void GraspManager::sendPick() {
  if (!ready_to_pick_) {
    ROS_INFO_THROTTLE(1, "Waiting for valid grasp pose...");
  } else {
    ROS_INFO("Picking!");
    ready_to_pick_ = false;
    pr2_picknplace_msgs::PickPlaceGoal pick;
    pick.goal.request = 0;
    pick.goal.header.frame_id = "base_link";
    pick.goal.header.stamp = ros::Time::now();
    pick.goal.object_pose.position = grasp_res_.averagedGraspPoint;
    // Kinect Offsets
    pick.goal.object_pose.position.x -= 0.035;
    pick.goal.object_pose.position.y -= 0.02;
    pick.goal.object_pose.position.z =
      std::max(0.68, pick.goal.object_pose.position.z - 0.055);

    // grasp_res_.roll = 2.35619; // Test fake roll

    // Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(grasp_res_.roll, Eigen::Vector3d::UnitZ());

    // Eigen::Quaternion<double> n_q = rollAngle * yawAngle * pitchAngle;

    // std::cout << "x:" << n_q.x()
    //           << " y:" << n_q.y()
    //           << " z:" << n_q.z()
    //           << " w:" << n_q.w();

    tf2::Quaternion q;    // TODO: WHY DOES IT ONLY WORK LIKE THIS?
    q.setRPY(0, 0, (grasp_res_.roll / M_PI) * 180);
    pick.goal.object_pose.orientation.x = q.w();
    pick.goal.object_pose.orientation.y = q.x();
    pick.goal.object_pose.orientation.z = q.y();
    pick.goal.object_pose.orientation.w = q.z();
    std::cout << "QUATERNION: X:" << q.x() <<
              " Y:" << q.y() <<
              " Z:" << q.z() <<
              " W:" << q.w();
    std::cout << pick.goal;
    if (execute_motion_) { pickplace_ac_.sendGoal(pick); }
    bool finished_before_timeout = pickplace_ac_.waitForResult(ros::Duration(10.0));
    ready_to_place_ = true;
  }
}

void GraspManager::sendPlace() {
  if (!ready_to_place_) {
    ROS_INFO_THROTTLE(1, "Waiting for object to be picked...");
  } else {
    ROS_INFO("Placing!");
    ready_to_place_ = false;
    pr2_picknplace_msgs::PickPlaceGoal place;
    place.goal.request = 1;
    place.goal.header.frame_id = "base_link";
    place.goal.header.stamp = ros::Time::now();
    place.goal.object_pose = place_pose_;
    std::cout << place.goal;
    if (execute_motion_) { pickplace_ac_.sendGoal(place); }
    bool finished_before_timeout = pickplace_ac_.waitForResult(ros::Duration(10.0));
    ready_to_grasp_ = true;
    request_pc_.call(empty_);
  }
}
