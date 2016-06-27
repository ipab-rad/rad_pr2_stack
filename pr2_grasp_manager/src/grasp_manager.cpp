/**
 * @file      grasp_manager.cpp
 * @brief     Provides grasping pipeline using haf_grasping and MoveIt
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-06-08
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <pr2_grasp_manager/grasp_manager.hpp>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <pr2_picknplace_msgs/GetSlope.h>

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
  ros::param::param(ns_ + "/max_ac_execution_time", max_ac_execution_time, 30.0);
  ros::param::param(ns_ + "/slope_max", slope_max, -10700.0);
}

void GraspManager::init() {
  pc_ready_ = false;
  ready_to_grasp_ = true;
  ready_to_pick_ = false;
  ready_to_place_ = false;
  eval_thresh = 0;

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

  place_pose_.position.x = 0.7;
  place_pose_.position.y = -0.45;
  place_pose_.position.z = 0.8;
  place_pose_.orientation.x = 0;
  place_pose_.orientation.y = 0;
  place_pose_.orientation.z = 0;
  place_pose_.orientation.w = 1;

  moveto_pose_.position.x = 0.6;
  moveto_pose_.position.y = -0.32;
  moveto_pose_.position.z = 0.9;
  moveto_pose_.orientation.x = 0;
  moveto_pose_.orientation.y = 0;
  moveto_pose_.orientation.z = 0;
  moveto_pose_.orientation.w = 1;

  // Different softness places
  place_pose_soft = place_pose_; place_pose_soft.position.x += 0.0;
  place_pose_semi_soft = place_pose_;
  place_pose_hard = place_pose_; place_pose_hard.position.x -= 0.1;


}

void GraspManager::rosSetup() {
  // point_cloud_sub_ = nh_.subscribe("/plane_segmentator/extracted_outliers",
  point_cloud_sub_ = nh_.subscribe("/plane_segmentator/segmented_objects_above",
                                   10, &GraspManager::pcCB, this);
  request_pc_ = nh_.serviceClient<std_srvs::Empty>
                ("/plane_segmentator/request_pointcloud", true);
  new_grasp_start_client = nh_.serviceClient<std_srvs::Empty>
                           ("/stiffness_estimator/new_grasp_start", true);
  get_slope_client = nh_.serviceClient<pr2_picknplace_msgs::GetSlope>
                     ("/stiffness_estimator/get_slope", true);
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

bool GraspManager::getGrasp() {
  bool success = false;
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
      if (grasp_res_.eval > eval_thresh) {
        ready_to_pick_ = true;
        success = true;
        ROS_INFO_STREAM("GetGrasp status: " << ((success) ? "success" : "fail"));
      } else {request_pc_.call(empty_); ready_to_grasp_ = true; }
    } else {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
  return success;
}

bool GraspManager::sendPick() {
  bool success = false;
  if (!ready_to_pick_) {
    ROS_INFO_THROTTLE(1, "Waiting for valid grasp pose...");
  } else {
    ROS_INFO("Clearing old grasping data.");
    new_grasp_start_client.call(empty_);
    ROS_INFO("Picking!");
    ready_to_pick_ = false;
    pr2_picknplace_msgs::PickPlaceGoal pick;
    pick.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PICK_REQUEST;
    pick.goal.header.frame_id = "base_link";
    pick.goal.header.stamp = ros::Time::now();
    pick.goal.object_pose.position = grasp_res_.averagedGraspPoint;

    // Kinect Offsets
    pick.goal.object_pose.position.x -= 0.031;
    // pick.goal.object_pose.position.y -= 0.02;
    pick.goal.object_pose.position.z =
      std::max(0.680, pick.goal.object_pose.position.z - 0.055);

    tf2::Quaternion q;    // TODO: WHY DOES IT ONLY WORK LIKE THIS?
    q.setRPY(0, 0, (grasp_res_.roll / M_PI) * 180);
    pick.goal.object_pose.orientation.x = q.w();
    pick.goal.object_pose.orientation.y = q.x();
    pick.goal.object_pose.orientation.z = q.y();
    pick.goal.object_pose.orientation.w = q.z();
    // std::cout << "QUATERNION: X:" << q.x() <<
    //           " Y:" << q.y() <<
    //           " Z:" << q.z() <<
    //           " W:" << q.w();

    std::cout << pick.goal;
    pickplace_ac_.sendGoal(pick);
    bool finished_before_timeout = pickplace_ac_.waitForResult(
                                     ros::Duration(max_ac_execution_time));
    if (finished_before_timeout) {
      ROS_INFO("Finished on time!");
      success = pickplace_ac_.getResult()->success;
    } else {
      ROS_WARN("Couldn't sendPick in timeframe of %fs!", max_ac_execution_time);
    }
    ROS_INFO_STREAM("Pick status: " << ((success) ? "success" : "fail"));
    if (success) {
      pr2_picknplace_msgs::GetSlope slope_msg;
      get_slope_client.call(slope_msg);
      last_slope = slope_msg.response.slope;
      ROS_WARN_STREAM("Raw slope: " << last_slope);
    }
    ready_to_place_ = true;
  }
  return success;
}

bool GraspManager::sendPlace(PLACE_LOCATION loc) {
  bool success = false;
  if (!ready_to_place_) {
    ROS_INFO_THROTTLE(1, "Waiting for object to be picked...");
  } else {
    ROS_INFO("Placing!");
    ready_to_place_ = false;
    pr2_picknplace_msgs::PickPlaceGoal place;
    place.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PLACE_REQUEST;
    place.goal.header.frame_id = "base_link";
    place.goal.header.stamp = ros::Time::now();
    switch (loc) {
      case PLACE_LOCATION::SOFT:
        ROS_WARN("Placing in a SOFT location");
        place.goal.object_pose = place_pose_soft;
        break;
      case PLACE_LOCATION::SEMI_SOFT:
        place.goal.object_pose = place_pose_semi_soft;
        break;
      case PLACE_LOCATION::HARD:
        ROS_WARN("Placing in a HARD location");
        place.goal.object_pose = place_pose_hard;
        break;
      default:
        ROS_WARN("Wrong location selected!");
        place.goal.object_pose = place_pose_;
    }
    std::cout << place.goal;
    pickplace_ac_.sendGoal(place);
    bool finished_before_timeout = pickplace_ac_.waitForResult(
                                     ros::Duration(max_ac_execution_time));
    success = pickplace_ac_.getResult()->success;
    ready_to_grasp_ = true;
    request_pc_.call(empty_);
  }
  return success;
}

bool GraspManager::sendMoveTo() {
  bool success = true;
  ROS_INFO("Moving arm out of the way!");
  pr2_picknplace_msgs::PickPlaceGoal place;
  place.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::MOVETO_REQUEST;
  place.goal.header.frame_id = "base_link";
  place.goal.header.stamp = ros::Time::now();
  place.goal.object_pose = moveto_pose_;
  std::cout << place.goal;
  pickplace_ac_.sendGoal(place);
  success &= pickplace_ac_.waitForResult(ros::Duration(max_ac_execution_time));
  success &= pickplace_ac_.getResult()->success;
  request_pc_.call(empty_);
  ready_to_grasp_ = true;
  return success;
}

double GraspManager::isSoft() {
  return double(last_slope) / slope_max;
}
