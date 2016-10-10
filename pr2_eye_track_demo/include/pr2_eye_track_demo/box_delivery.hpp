/**
 * @file      pr2_box_delivery.hpp
 * @brief     Provides basic pick up and place of boxes
 * @author    Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-10-04
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#ifndef BOX_DELIVERY_HPP
#define BOX_DELIVERY_HPP

// System
#include <string>
#include <map>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>

// Services
#include <std_srvs/Empty.h>
#include <pr2_eye_track_demo/BoxQuery.h>

typedef actionlib::SimpleActionClient <pr2_picknplace_msgs::PickPlaceAction>
PickPlaceAC;
typedef std::shared_ptr<PickPlaceAC> PickPlaceACPtr;

class BoxDelivery {
  public:
    BoxDelivery(ros::NodeHandle* nh);
    ~BoxDelivery(void);

    void loadParams();
    void init();
    void rosSetup();

    void updatePose(std::string frame, geometry_msgs::TransformStamped p);

  private:
    // ROS
    ros::NodeHandle* nh_;
    ros::ServiceServer pick_up_service;

    PickPlaceACPtr pp_left;
    PickPlaceACPtr pp_right;

    // Variables
    std::string ns_;
    double max_planning_time_;
    double max_time_box_unobservable_;

    std::map<std::string, geometry_msgs::TransformStamped> box_poses;

    // Methods
    bool pick_up_callback(pr2_eye_track_demo::BoxQuery::Request& request,
                          pr2_eye_track_demo::BoxQuery::Response& response);
};

#endif  /* BOX_DELIVERY_HPP */
