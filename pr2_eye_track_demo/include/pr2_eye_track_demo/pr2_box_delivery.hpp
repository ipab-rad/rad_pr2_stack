/**
 * @file      pr2_box_delivery.hpp
 * @brief     Provides basic pick up and place of boxes
 * @author    Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-10-04
 * @copyright (MIT) 2016 RAD-UoE Informatics
 */

#ifndef PR2_BOX_DELIVERY_HPP
#define PR2_BOX_DELIVERY_HPP

// System
#include <string>
// #include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>

// Services
#include <std_srvs/Empty.h>

class PR2BoxDelivery {
  public:
    PR2BoxDelivery(ros::NodeHandle* nh);
    ~PR2BoxDelivery(void);

    void loadParams();
    void init();
    void rosSetup();

  private:
    // ROS
    ros::NodeHandle* nh_;
    ros::ServiceServer pick_up_service;

    // Variables
    std::string ns_;
    double max_planning_time_;

    // Methods
    bool pick_up_callback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
};

#endif  /* PR2_BOX_DELIVERY_HPP */
