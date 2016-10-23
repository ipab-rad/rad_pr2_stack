/**
 * @file      cooking.hpp
 * @brief     Provides basic manipulation of objects
 * @author    Yordan Hristov <yordan.hristov@ed.ac.uk>
 * @date      2016-10-17
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#include "pr2_cooking_demo/cooking.hpp"

Cooking::Cooking(ros::NodeHandle* nh):
    nh_(nh) {

    ROS_INFO("Starting Cooking!");
    this->loadParams();
    this->init();
    this->rosSetup();
    ROS_INFO("Finished startup");
}

Cooking::~Cooking() {
    ros::param::del(ns_);
}

void Cooking::loadParams() {
    ns_ = "pr2_cooking_demo_node";
    ROS_INFO("[Cooking] Loading parameters.");
    ROS_INFO_STREAM("[Cooking] NAMESPACE: " << ns_);
    if (!ros::param::get(ns_ + "/max_planning_time", max_planning_time_)) {
        ROS_WARN("[Cooking] Parameters were not loaded! Using default.");
    }
    ros::param::param(ns_ + "/max_planning_time", max_planning_time_, 10.0);
    ros::param::param<std::string>(ns_ + "/banana_and_apple_phrase", banana_and_apple_phrase, "default");
    ros::param::param<std::string>(ns_ + "/use_bowl_phrase", use_bowl_phrase, "default");
    ros::param::param<std::string>(ns_ + "/ok_making_salad_phrase", ok_making_salad_phrase, "default");
    ros::param::param<std::string>(ns_ + "/no_banana_use_orange_phrase", no_banana_use_orange_phrase, "default");
}

void Cooking::init() {
    state = 0;

    bowl_offset.position.x = 0.0;
    bowl_offset.position.y = 0.04;
    bowl_offset.position.z = 0.05;
    bowl_offset.orientation.x = 0.707;
    bowl_offset.orientation.y = 0.0;
    bowl_offset.orientation.z = 0.0;
    bowl_offset.orientation.w = 0.707;

    fruit_offset.position.x = 0.0;
    fruit_offset.position.y = 0.03;
    fruit_offset.position.z = 0.002;
    fruit_offset.orientation.x = 0.707;
    fruit_offset.orientation.y = 0.0;
    fruit_offset.orientation.z = 0.0;
    fruit_offset.orientation.w = 0.707;

    left_arm_offset.position.x = 0.3;
    left_arm_offset.position.z = 0.9;
    left_arm_offset.position.y = 0.5;
    left_arm_offset.orientation.x = 0.0;
    left_arm_offset.orientation.y = 0.0;
    left_arm_offset.orientation.z = 0.0;
    left_arm_offset.orientation.w = 1.0;

    right_arm_offset.position.x = 0.3;
    right_arm_offset.position.z = 0.9;
    right_arm_offset.position.y = -0.5;
    right_arm_offset.orientation.x = 0.0;
    right_arm_offset.orientation.y = 0.0;
    right_arm_offset.orientation.z = 0.0;
    right_arm_offset.orientation.w = 1.0;
}

void Cooking::rosSetup() {

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

    look_pub = nh_->advertise<pr2_head_msgs::LookAt>("/pr2_head/target_object", 100, true);
    ROS_INFO("Publisher to '/pr2_head/target_object' initialized");
    say_service = nh_->serviceClient<pr2_head::Query>("/pr2_head/say");
    ROS_INFO("Service client for '/pr2_head/say' initialized");
}

bool Cooking::manipulateObject(
    std::string frame,
    uint request,
    std::string arm) {

    std::string msg;
    pr2_picknplace_msgs::PickPlaceGoal pick;
    pick.goal.request = request;
    pick.goal.header.frame_id = frame;
    if (frame == "base_link") {
        if (arm == "left")
            pick.goal.object_pose = left_arm_offset;
        else 
            pick.goal.object_pose = right_arm_offset;
    }
    else if (frame == "bowl1" || frame == "bowl2")
        pick.goal.object_pose = bowl_offset;   
    else if (frame == "banana") {
        fruit_offset.position.y = 0.1;
        pick.goal.object_pose = fruit_offset;
    }
    else {
        fruit_offset.position.y = 0.03;                
        pick.goal.object_pose = fruit_offset;    
    }

    PickPlaceACPtr pp;
    if (arm == "left") {
        pp = pp_left;
    } else if (arm == "right") {
        pp = pp_right;
    } else {
        ROS_WARN_STREAM("Wrong arm specifier: " << arm << "Use 'left' or 'right'.");
        return false;
    }

    ROS_INFO("Sending pick request!");
    pp->sendGoal(pick);
    ROS_INFO_STREAM("Pick frame: " << pick.goal.header.frame_id);

    pp->waitForResult(ros::Duration(max_planning_time_));
    if (pp->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Didn't finish pick request in time!");
        return false;
    }

    ROS_DEBUG_STREAM("Stat: " << int(pp->getResult()->success));
    if (!pp->getResult()->success) {
        ROS_INFO("Couldn't execute pick request");
        return false;
    } 

    ROS_INFO_STREAM("Finished pick request successfully for frame " << frame);
    return true;
}

bool Cooking::pickSimFruits(
    std::string frame_left,
    std::string frame_right) {

    pr2_picknplace_msgs::PickPlaceGoal pick_left;
    pr2_picknplace_msgs::PickPlaceGoal pick_right;

    pick_left.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PICK_REQUEST;
    pick_left.goal.header.frame_id = frame_left;

    pick_right.goal.request = pr2_picknplace_msgs::PicknPlaceGoal::PICK_REQUEST;
    pick_right.goal.header.frame_id = frame_right;
    
    pick_right.goal.object_pose = fruit_offset;

    if (frame_left == "banana") {
        fruit_offset.position.y = 0.1;
        pick_left.goal.object_pose = fruit_offset;
    }
    else {
        fruit_offset.position.y = 0.03;                
        pick_left.goal.object_pose = fruit_offset;   
    }

    ROS_INFO("Sending pick requests!");
    pp_left->sendGoal(pick_left);
    pp_right->sendGoal(pick_right);

    ROS_INFO_STREAM("Pick frame: " << pick_left.goal.header.frame_id);
    ROS_INFO_STREAM("Pick frame: " << pick_right.goal.header.frame_id);

    pp_left->waitForResult(ros::Duration(max_planning_time_));
    pp_right->waitForResult(ros::Duration(max_planning_time_));

    if (pp_left->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Didn't finish pick request in time!");
        return false;
    }

    ROS_DEBUG_STREAM("Stat: " << int(pp_left->getResult()->success));
    if (!pp_left->getResult()->success) {
        ROS_INFO("Couldn't execute pick request");
        return false;
    }

    if (pp_right->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Didn't finish pick request in time!");
        return false;
    }

    ROS_DEBUG_STREAM("Stat: " << int(pp_right->getResult()->success));
    if (!pp_right->getResult()->success) {
        ROS_INFO("Couldn't execute pick request");
        return false;
    } 

    ROS_INFO_STREAM("Finished pick request successfully for frame " << frame_left);
    ROS_INFO_STREAM("Finished pick request successfully for frame " << frame_right);
    return true;
}

int Cooking::get_state() {
    return state;
}

void Cooking::set_state(int state) {
    this->state = state; 
}

void Cooking::combine_fruits(std::string fruit1, std::string fruit2, std::string bowl) {

    // Pick the fruits
    look_at(fruit1);
    pickSimFruits(fruit1, fruit2);

    look_at(bowl);
    manipulateObject(bowl,
                     pr2_picknplace_msgs::PicknPlaceGoal::PLACE_REQUEST, "right");
    manipulateObject("base_link",
                     pr2_picknplace_msgs::PicknPlaceGoal::MOVETO_REQUEST, "right");

    bowl_offset.position.y = 0.1;

    manipulateObject(bowl,
                     pr2_picknplace_msgs::PicknPlaceGoal::PLACE_REQUEST, "left");
    manipulateObject("base_link",
                     pr2_picknplace_msgs::PicknPlaceGoal::MOVETO_REQUEST, "left");

}

void Cooking::look_at(std::string object){
    pr2_head_msgs::LookAt look_query;
    geometry_msgs::Point point;

    if (object == "person") {
        look_query.frame = "base_link";
        look_query.follow = false;
        
        point.x = 1.0;
        point.y = -1.0;
        point.z = 1.2;
    }
    else if (object == "table") {
        look_query.frame = "base_link";
        look_query.follow = false;
        
        point.x = 0.5;
        point.y = 0.0;
        point.z = 0.6;
    }
    else {
        look_query.frame = object.c_str();
        look_query.follow = false;
        
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
    }
    look_query.offset = point;
    ROS_INFO("%s", look_query.frame.c_str());
    look_pub.publish(look_query);
}

void Cooking::say(std::string query){
    pr2_head::Query srv;
    srv.request.query = query;
    if (say_service.call(srv)) {
        ROS_INFO("PR2 just said %s", query.c_str());
    }
    else {
        ROS_ERROR("PR2 failed to speak");
    }
}