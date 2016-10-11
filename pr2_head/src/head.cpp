/**
 * @file      head.cpp
 * @brief     Provides basic looking at object frame functionality
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @modified  Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_head/head.hpp"

Head::Head(ros::NodeHandle& nh) :
  nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  point_head_client_ =
    new PointHeadClient("/head_traj_controller/point_head_action", true);
  speak_client_ = new SpeakClient("/sound_play", true);

  ROS_INFO("Waiting for point head action server...");
  ROS_WARN("Head will not move if joystick node is up! Please stop it now.");
  if (!point_head_client_->waitForServer(ros::Duration(max_waiting_time_))) {
    ROS_WARN("Couldn't reach the head trajectory controller. Head will not move.");
  }

  ROS_INFO("Waiting for sound play action server...");
  if (!speak_client_->waitForServer(ros::Duration(max_waiting_time_))) {
    ROS_WARN("Couldn't reach the speaker client. No audio will be played.");
  }
  ROS_INFO("PR2 Head initialised.");
}

Head::~Head() {
  ros::param::del(nh_.getNamespace());
  delete point_head_client_;
  delete speak_client_;
}

void Head::loadParams() {
  nh_.param<bool>("follow_object", follow_object_, true);
  nh_.param<bool>("look_table", look_table_, false);
  nh_.param<double>("table_pos/x", table_pos_.x, 0.0);
  nh_.param<double>("table_pos/y", table_pos_.y, 0.0);
  nh_.param<double>("table_pos/z", table_pos_.z, 0.0);
  nh_.param<bool>("vocally_declare_object", vocally_declare_object_, false);
  nh_.param<double>("max_waiting_time", max_waiting_time_, 2.0);
}

void Head::init() {
  target_object_ = "";
  look_at_object_ = "";
  target_offset_.x = 0.0;
  target_offset_.y = 0.0;
  target_offset_.z = 0.0;
}

void Head::rosSetup() {
  target_object_sub_ = nh_.subscribe("target_object", 1, &Head::objCB, this);
  talk_srv_ = nh_.advertiseService("say", &Head::say, this);
  shake_srv_ = nh_.advertiseService("shake", &Head::shakeCB, this);
  nod_srv_ = nh_.advertiseService("nod", &Head::nodCB, this);
}

void Head::objCB(const pr2_head_msgs::LookAt::Ptr msg) {
  ROS_INFO_STREAM("Last target object: " << target_object_);
  target_object_ = msg->frame;
  target_offset_ = msg->offset;
  ROS_INFO_STREAM("New target object : " << target_object_);
}

void Head::ready() {
  this->speak("PR2 Ready");
  if (look_table_)
  {this->lookAt("base_link", table_pos_);}
}

void Head::updateLookingPosition() {
  bool new_obj = target_object_ != look_at_object_;
  if (target_object_ != "" && target_object_.compare("none") != 0) {
    if ((new_obj) || (follow_object_ == true) ) {
      look_at_object_ = target_object_;
      if (new_obj && vocally_declare_object_) {
        std::string sentence = "Looking at " + look_at_object_;
        this->speak(sentence);
      }
      this->lookAt(look_at_object_, target_offset_);
    }
  }
}

bool Head::lookAt(std::string frame_id, geometry_msgs::Point offset) {
  pr2_controllers_msgs::PointHeadGoal goal;

  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = offset.x; 
  point.point.y = offset.y; 
  point.point.z = offset.z;
  goal.target = point;

  goal.pointing_frame = "head_plate_frame"; //"high_def_frame";
  goal.pointing_axis.x = 1;  // pointing_axis defaults to X-axis
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;

  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.5;  //  rad/sec

  point_head_client_->sendGoal(goal);
  return point_head_client_->waitForResult(ros::Duration(max_waiting_time_));
}

void Head::speak(std::string sentence) {
  sound_play::SoundRequestGoal goal;
  sound_play::SoundRequest sound_request;

  sound_request.sound = -3;
  sound_request.command = 1;
  sound_request.arg = sentence;

  goal.sound_request = sound_request;
  speak_client_->sendGoal(goal);
}

bool Head::say(pr2_head::Query::Request&  req,
               pr2_head::Query::Response& res) {
  this->speak(req.query);
  res.success = true;
  return true;
}

void Head::shake(uint n) {
  uint count = 0;
  geometry_msgs::Point point;
  point.x = 5.0;
  point.y = 1.0;
  point.z = 1.2;
  while (ros::ok() && ++count <= n ) {
    this->lookAt("base_link", point);
    point.y = -1.0;
    this->lookAt("base_link", point);
  }
  updateLookingPosition();
}

void Head::nod(uint n) {
  uint count = 0;
  geometry_msgs::Point point;
  point.x = 5.0;
  point.y = 0.0;
  point.z = -1.0;
  while (ros::ok() && ++count <= n ) {
    this->lookAt("base_link", point);
    point.z = 2.0;
    this->lookAt("base_link", point);
  }
  updateLookingPosition();
}

bool Head::shakeCB(std_srvs::Empty::Request& req,
                   std_srvs::Empty::Response& res) {
  shake(2);
  return true;
}

bool Head::nodCB(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res) {
  nod(2);
  return true;
}
