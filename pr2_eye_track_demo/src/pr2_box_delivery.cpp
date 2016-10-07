/**
 * @file      pr2_box_delivery.cpp
 * @brief     Provides basic pick up and place of boxes
 * @author    Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-10-04
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#include "pr2_eye_track_demo/pr2_box_delivery.hpp"
#include <std_srvs/Empty.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>

PR2BoxDelivery::PR2BoxDelivery(ros::NodeHandle* nh):
    nh_(nh) {

    ROS_INFO("Starting PR2BoxDelivery!");
    this->loadParams();
    this->init();
    this->rosSetup();
    ROS_INFO("Finished startup");
}

PR2BoxDelivery::~PR2BoxDelivery() {
    ros::param::del(ns_);
}

void PR2BoxDelivery::loadParams() {
    ROS_INFO("[PR2BoxDelivery] Loading parameters.");
    ROS_INFO_STREAM("[PR2BoxDelivery] NAMESPACE: " << ns_);
    if (!ros::param::get(ns_ + "/max_planning_time", max_planning_time_)) {
        ROS_WARN("[PR2BoxDelivery] Parameters were not loaded! Using default.");
    }
    ros::param::param(ns_ + "/max_planning_time", max_planning_time_, 10.0);
    ros::param::param(ns_ + "/max_time_box_unobservable",
                      max_time_box_unobservable_, 10.0);
}

void PR2BoxDelivery::init() {
}

void PR2BoxDelivery::rosSetup() {
    pick_up_service =
        nh_->advertiseService("pick_up_box", &PR2BoxDelivery::pick_up_callback, this);
}


bool PR2BoxDelivery::pick_up_callback(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
    ROS_INFO("Picking up box!");
    return true;
}


void PR2BoxDelivery::updatePose(std::string frame,
                                geometry_msgs::TransformStamped p) {
    box_poses[frame] = p;
    ROS_INFO_STREAM("Updated pose for frame: " << frame);
}
