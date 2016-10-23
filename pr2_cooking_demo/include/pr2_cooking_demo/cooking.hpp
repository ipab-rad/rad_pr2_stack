/**
 * @file      cooking.hpp
 * @brief     Provides basic manipulation of objects
 * @author    Yordan Hristov <yordan.hristov@ed.ac.uk>
 * @date      2016-10-17
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#ifndef COOKING_HPP
#define COOKING_HPP

// System
#include <string>
#include <map>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <pr2_head_msgs/LookAt.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <pr2_head/Query.h>
#include <geometry_msgs/Point.h>

// Services

typedef actionlib::SimpleActionClient <pr2_picknplace_msgs::PickPlaceAction> PickPlaceAC;
typedef std::shared_ptr<PickPlaceAC> PickPlaceACPtr;

class Cooking {
  public:
    Cooking(ros::NodeHandle* nh);
    ~Cooking(void);

    void loadParams();
    void init();
    void rosSetup();

    void combine_fruits(std::string fruit1, std::string fruit2, std::string bowl);
    int get_state();
    void set_state(int state);
    void look_at(std::string object);
    void say(std::string query);

    std::string banana_and_apple_phrase, use_bowl_phrase, 
                ok_making_salad_phrase, no_banana_use_orange_phrase;

  private:
    // ROS
    ros::NodeHandle* nh_;

    PickPlaceACPtr pp_left;
    PickPlaceACPtr pp_right;

    ros::ServiceClient say_service;
    ros::Publisher look_pub;

    // Variables
    std::string ns_;
    double max_planning_time_;

    int state;
    geometry_msgs::Pose bowl_offset, left_arm_offset, right_arm_offset, fruit_offset;

    // Methods
    bool manipulateObject(
        std::string frame,
        uint request,
        std::string arm);

    bool pickSimFruits(
    std::string frame_left,
    std::string frame_right);

};

#endif  /* COOKING_HPP */