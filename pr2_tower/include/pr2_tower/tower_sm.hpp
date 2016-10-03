/**
 * @file      pr2_tower.hpp
 * @brief     PR2 Tower Building State Machine
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-09-01
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef TOWER_SM_HPP
#define TOWER_SM_HPP

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <pr2_tower_msgs/TowerSMAction.h>
#include <pr2_tower_msgs/TowerSMGoal.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <actionlib/client/simple_action_client.h>

class TowerSM {
 protected:
	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line.
	// Otherwise strange error may occur.
	actionlib::SimpleActionServer<pr2_tower_msgs::TowerSMAction> as_;
	std::string action_name_;
	pr2_tower_msgs::TowerSMFeedback feedback_;
	pr2_tower_msgs::TowerSMResult result_;

 public:
	TowerSM(ros::NodeHandle& nh, std::string name);
	~TowerSM(void);

	void loadParams();
	void init();
	void rosSetup();

	void goalCB();

	void preemptCB();

	void executeCB();

 private:
	// Methods
	bool buildTower();
	bool moveArmsAway();
	bool calibrateBlocks();
	bool dissassembleTower();
	bool releaseTower();
	bool checkOK();

	// Flags

	// Parameters
	float ac_timeout_;
	bool hold_tower_;
	int num_blocks_;
	float block_height_;
	geometry_msgs::Pose tower_pose_;

	// Variables
	std::vector<geometry_msgs::Pose> block_poses_;
	pr2_tower_msgs::TowerSMGoal tower_sm_goal_;
	pr2_picknplace_msgs::PickPlaceGoal pick_;
	pr2_picknplace_msgs::PickPlaceGoal place_;
	pr2_picknplace_msgs::PickPlaceGoal movetoright_;
	pr2_picknplace_msgs::PickPlaceGoal movetoleft_;
	pr2_picknplace_msgs::PickPlaceGoal hold_;
	pr2_picknplace_msgs::PickPlaceGoal release_;

	// ROS
	actionlib::SimpleActionClient<pr2_picknplace_msgs::PickPlaceAction> ac_right_;
	actionlib::SimpleActionClient<pr2_picknplace_msgs::PickPlaceAction> ac_left_;

};

#endif  /* TOWER_SM_HPP */
