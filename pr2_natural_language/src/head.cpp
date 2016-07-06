/**
 * @file      head.cpp
 * @brief     Provides basic looking at object frame functionality
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_natural_language/head.hpp"

Head::Head(ros::NodeHandle& nh) :
  nh_(nh)
  // tfListener(tfBuffer)
{
  this->loadParams();
  this->init();
  this->rosSetup();
  point_head_client_ =
    new PointHeadClient("/head_traj_controller/point_head_action", true);
  speak_client_ = new SpeakClient("/sound_play", true);
  ROS_INFO("Waiting for point head action server...");
  point_head_client_->waitForServer();
  ROS_INFO("Waiting for sound play action server...");
  speak_client_->waitForServer();
  ROS_INFO("PR2 Natural Language initialised.");
}

Head::~Head() {
  ros::param::del(nh_.getNamespace());
  delete point_head_client_;
  delete speak_client_;
}

void Head::loadParams() {
  nh_.getParam("follow_object", follow_object_);
  nh_.getParam("table_pos/x", table_pos_.x);
  nh_.getParam("table_pos/y", table_pos_.y);
  nh_.getParam("table_pos/z", table_pos_.z);
}

void Head::init() {
  target_object_ = "";
  look_at_object_ = "";
}

void Head::rosSetup() {
  target_object_sub_ = nh_.subscribe("target_object", 1, &Head::objCB, this);
  talk_srv_ = nh_.advertiseService("say", &Head::say, this);
}

void Head::objCB(const std_msgs::String::Ptr msg) {
  ROS_INFO_STREAM("TargObj: " << target_object_);
  target_object_ = msg->data;
  ROS_INFO_STREAM("TargObj: " << target_object_);
}

void Head::ready() {
  this->speak("PR2 Ready");
  this->lookAt("base_link", table_pos_.x, table_pos_.y, table_pos_.z);
}

void Head::update() {
  bool new_obj = target_object_ != look_at_object_;
  if (target_object_ != "") {
    if ((new_obj) || (follow_object_ == true) ) {
      look_at_object_ = target_object_;
      if (new_obj) {
        // std::string sentence = "Looking at " + look_at_object_;
        // this->speak(sentence);
      }
      this->lookAt(look_at_object_, 0.0, 0.0, 0.0);
    }
  }
}

void Head::lookAt(std::string frame_id, double x, double y, double z) {
  pr2_controllers_msgs::PointHeadGoal goal;

  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal.target = point;

  goal.pointing_frame = "high_def_frame";
  // goal.pointing_frame = "head_mount_kinect2_rgb_link";
  goal.pointing_axis.x = 1;  // (pointing_axis defaults to X-axis)
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;

  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.5;  //  rad/sec

  point_head_client_->sendGoal(goal);
  point_head_client_->waitForResult(ros::Duration(2));
}

void Head::speak(std::string sentence) {
  sound_play::SoundRequestGoal goal;
  sound_play::SoundRequest sound_request;
  sound_request.sound = -3;
  sound_request.command = 1;
  sound_request.arg = sentence;
  // goal.sound_request.arg2 = "";

  goal.sound_request = sound_request;
  speak_client_->sendGoal(goal);
}

bool Head::say(pr2_natural_language::Query::Request&  req,
               pr2_natural_language::Query::Response& res) {
  this->speak(req.query);
  res.success = true;
  return true;
}

void Head::shake(uint n) {
  uint count = 0;
  while (ros::ok() && ++count <= n ) {
    this->lookAt("base_link", 5.0, 1.0, 1.2);
    this->lookAt("base_link", 5.0, -1.0, 1.2);
  }
}
