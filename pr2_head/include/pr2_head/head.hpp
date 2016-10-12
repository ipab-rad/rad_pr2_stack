/**
 * @file      head.hpp
 * @brief     Provides basic looking at object frame functionality
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @modified  Daniel Angelov <d.angelov@ed.ac.uk>
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
#include <pr2_head/Query.h>
#include <std_srvs/Empty.h>
 #include <pr2_head_msgs/LookAt.h>

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

  void ready();
  void updateLookingPosition();
  bool lookAt(std::string frame_id, geometry_msgs::Point offset);
  void speak(std::string sentence);
  bool say(pr2_head::Query::Request&  req,
           pr2_head::Query::Response& res);
  void shake(uint n);
  void nod(uint n);

 private:
  // Methods
  void loadParams();
  void init();
  void rosSetup();
  void objCB(const pr2_head_msgs::LookAt::Ptr msg);
  bool shakeCB(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& res);
  bool nodCB(std_srvs::Empty::Request& req,
             std_srvs::Empty::Response& res);

  // Flags

  // Parameters
  bool follow_object_;
  bool look_table_;
  geometry_msgs::Point table_pos_;

  // Variables
  std::string target_object_;
  geometry_msgs::Point target_offset_;
  std::string look_at_object_;
  bool vocally_declare_object_;
  double max_waiting_time_;

  // ROS
  ros::ServiceServer talk_srv_;
  ros::ServiceServer shake_srv_;
  ros::ServiceServer nod_srv_;
  ros::Subscriber target_object_sub_;
  PointHeadClient* point_head_client_;
  SpeakClient* speak_client_;
};

#endif  /* HEAD_HPP */
