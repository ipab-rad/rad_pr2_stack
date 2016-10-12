/**
 * @file      box_delivery.cpp
 * @brief     Provides basic pick up and place of boxes
 * @author    Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-10-04
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#include "pr2_eye_track_demo/box_delivery.hpp"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

geometry_msgs::Pose BoxDelivery::box_offset;

BoxDelivery::BoxDelivery(ros::NodeHandle* nh):
    nh_(nh) {

    ROS_INFO("Starting BoxDelivery!");
    this->loadParams();
    this->init();
    this->rosSetup();
    ROS_INFO("Finished startup");
}

BoxDelivery::~BoxDelivery() {
    ros::param::del(ns_);
}

void BoxDelivery::loadParams() {
    ROS_INFO("[BoxDelivery] Loading parameters.");
    ROS_INFO_STREAM("[BoxDelivery] NAMESPACE: " << ns_);
    if (!ros::param::get(ns_ + "/max_planning_time", max_planning_time_)) {
        ROS_WARN("[BoxDelivery] Parameters were not loaded! Using default.");
    }
    ros::param::param(ns_ + "/max_planning_time", max_planning_time_, 10.0);
    ros::param::param(ns_ + "/max_time_box_unobservable",
                      max_time_box_unobservable_, 10.0);
}

void BoxDelivery::init() {
    box_offset.position.x = 0.0;
    box_offset.position.y = 0.0;
    box_offset.position.z = 0.2;

    box_offset.orientation.x = 0.0;
    box_offset.orientation.y = 0.0;
    box_offset.orientation.z = 0.0;
    box_offset.orientation.w = 1.0;
}

void BoxDelivery::rosSetup() {
    pick_up_service =
        nh_->advertiseService("pick_up_box", &BoxDelivery::pick_up_callback, this);
    put_down_service =
        nh_->advertiseService("put_down_box", &BoxDelivery::put_down_callback, this);

    double wait_time = 10;
    pp_left = PickPlaceACPtr(new PickPlaceAC("/pr2_picknplace_left/pr2_picknplace",
                                             true));
    while (!pp_left->waitForServer(ros::Duration(wait_time))) {
        ROS_DEBUG_THROTTLE(wait_time,
                           "Waiting for action client 'pr2_picknplace_left/pr2_picknplace'");
    }
    ROS_INFO("Action client '/pr2_picknplace_left/pr2_picknplace' initialized");

    pp_right = PickPlaceACPtr(
                   new PickPlaceAC("/pr2_picknplace_right/pr2_picknplace", true));
    while (!pp_right->waitForServer(ros::Duration(wait_time))) {
        ROS_DEBUG_THROTTLE(wait_time,
                           "Waiting for action client 'pr2_picknplace_right/pr2_picknplace'");
    }
    ROS_INFO("Action client '/pr2_picknplace_right/pr2_picknplace' initialized");
}

bool BoxDelivery::pickUp(
    std::string frame,
    geometry_msgs::Pose offset,
    std::string arm,
    std::string& msg) {

    geometry_msgs::Pose loc;
    if (box_poses[frame].header.frame_id == "base_link") {

        loc.position.x = box_poses[frame].transform.translation.x +
                         offset.position.x;
        loc.position.y = box_poses[frame].transform.translation.y +
                         offset.position.y;
        loc.position.z = box_poses[frame].transform.translation.z +
                         offset.position.z;

        loc.orientation.x = box_poses[frame].transform.rotation.x;
        loc.orientation.y = box_poses[frame].transform.rotation.y;
        loc.orientation.z = box_poses[frame].transform.rotation.z;
        loc.orientation.w = box_poses[frame].transform.rotation.w;


        last_pickup_loc = loc;
        ROS_INFO_STREAM("Tf: " << loc);
    } else {
        ROS_WARN_STREAM("Wrong TF base frame! Currently '" <<
                        box_poses[frame].header.frame_id  <<
                        "'. Expecting 'base_link'.");
        msg = "Wrong TF base frame! Probably haven't seen the tag";
        return false;
    }

    pr2_picknplace_msgs::PickPlaceGoal pick;
    pick.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PICK_REQUEST;
    pick.goal.header.frame_id = "base_link";
    pick.goal.object_pose = loc;

    PickPlaceACPtr pp;
    if (arm == "left") {
        pp = pp_left;
    } else if (arm == "right") {
        pp = pp_right;
    } else {
        msg = "Wrong arm specifier: '" + arm + "'. Use 'left' or 'right'.";
        ROS_WARN_STREAM(msg);
        return false;
    }

    ROS_INFO("Sending pick request!");
    pp->sendGoal(pick);
    ROS_INFO_STREAM("Pick position: " << pick.goal.object_pose.position);

    pp->waitForResult(ros::Duration(max_planning_time_));
    if (pp->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        msg = "Didn't finish pick request in time!";
        return false;
    }

    ROS_DEBUG_STREAM("Stat: " << int(pp->getResult()->success));
    if (!pp->getResult()->success) {
        msg = "Couldn't execute pick request";
        return false;
    }

    msg = "Finished pick request successfully for frame " + frame;

    // TODO: Send box to humans face!
    ROS_INFO("Finished picking a box");
    return true;
}

bool BoxDelivery::pick_up_callback(
    pr2_eye_track_demo_msgs::BoxQuery::Request& request,
    pr2_eye_track_demo_msgs::BoxQuery::Response& response) {
    ROS_INFO("Picking up box!");

    std::string msg;
    response.success = pickUp(request.box_frame, box_offset, request.arm, msg);
    response.message = msg;

    return true;
}

bool BoxDelivery::put_down_callback(
    std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response) {

    pr2_picknplace_msgs::PickPlaceGoal place;
    place.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PLACE_REQUEST;
    place.goal.header.frame_id = "base_link";
    place.goal.object_pose = last_pickup_loc;

    pp_left->sendGoal(place);
    ROS_INFO_STREAM("Place position: " << place.goal.object_pose.position);

    pp_left->waitForResult(ros::Duration(max_planning_time_));
    if (pp_left->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        response.success = false;
        response.message = "Didn't finish place request in time!";
        return true;
    }

    ROS_DEBUG_STREAM("Stat: " << int(pp_left->getResult()->success));
    if (!pp_left->getResult()->success) {
        response.success = false;
        response.message = "Couldn't execute place request";
        return true;
    }

    return true;
}

void BoxDelivery::updatePose(std::string frame,
                             geometry_msgs::TransformStamped p) {
    box_poses[frame] = p;
    ROS_INFO_STREAM_THROTTLE(1, "Updated pose for frame: " << frame);
}
