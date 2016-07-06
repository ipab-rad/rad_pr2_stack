/**
 * @file      head.hpp
 * @brief     Provides basic looking at object frame functionality
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-02-06
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef HEAD_HPP
#define HEAD_HPP

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <std_msgs/String.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <sound_play/SoundRequestAction.h>
#include <pr2_natural_language/Query.h>

typedef actionlib::SimpleActionClient
<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient
<sound_play::SoundRequestAction> SpeakClient;

class Head {
 protected:
  ros::NodeHandle nh_;

 public:
  Head(ros::NodeHandle& nh);
  ~Head(void);

  void loadParams();
  void init();
  void rosSetup();

  void objCB(const std_msgs::String::Ptr msg);

  void ready();
  void update();
  void lookAt(std::string frame_id, double x, double y, double z);
  void speak(std::string sentence);
  bool say(pr2_natural_language::Query::Request&  req,
           pr2_natural_language::Query::Response& res);
  void shake(uint n);

 private:
  // Methods

  // Flags
  bool follow_object_;

  // Parameters
  geometry_msgs::Vector3 table_pos_;

  // Variables
  std::string target_object_;
  std::string look_at_object_;

  // ROS
  ros::ServiceServer talk_srv_;
  ros::Subscriber target_object_sub_;
  PointHeadClient* point_head_client_;
  SpeakClient* speak_client_;
};

#endif  /* HEAD_HPP */
