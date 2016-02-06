/**
 * @file      pick_place.cpp
 * @brief     Add file description...
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "pr2_picknplace/pick_place.hpp"

PickPlaceAction::PickPlaceAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PickPlaceAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PickPlaceAction::preemptCB, this));

  ROS_INFO("Starting PickPlace action server");
  this->loadParams();
  this->init();
  this->rosSetup();
}

PickPlaceAction::~PickPlaceAction() {
  ros::param::del("pick_place");
}

void PickPlaceAction::loadParams() {
}

void PickPlaceAction::init() {
}

void PickPlaceAction::rosSetup() {
}

void PickPlaceAction::goalCB() {
  request_ = as_.acceptNewGoal()->request;
  object_pose_ = as_.acceptNewGoal()->object_pose;
  this->executeCB();
}

void PickPlaceAction::preemptCB() {
  going = false;
  as_.setPreempted();
}

void PickPlaceAction::executeCB() {
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  bool going = true;
  bool success = true;

  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    success = false;
    going = false;
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
