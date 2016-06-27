/**
 * @file      grasp_manager.hpp
 * @brief     Provides grasping pipeline using haf_grasping and MoveIt
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-06-08
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef GRASP_MANAGER_HPP
#define GRASP_MANAGER_HPP

#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

// Haf Grasping
#include <haf_grasping/GraspInput.h>
#include <haf_grasping/GraspOutput.h>
#include <haf_grasping/CalcGraspPointsServerAction.h>

// PicknPlace
#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>

enum PLACE_LOCATION {
	SOFT = 0,
	SEMI_SOFT = 1,
	HARD = 2
};

class GraspManager {
  protected:
	void loadParams();
	void init();
	void rosSetup();

	ros::NodeHandle nh_;

  public:
	GraspManager(ros::NodeHandle& nh);
	~GraspManager(void);

	bool getGrasp();
	bool sendPick();
	bool sendPlace(PLACE_LOCATION loc = PLACE_LOCATION::HARD);
	bool sendMoveTo();
	double isSoft();

  private:
	// Methods
	void pcCB(const sensor_msgs::PointCloud2ConstPtr& msg);

	// Flags
	bool pc_ready_;
	bool ready_to_grasp_;
	bool ready_to_pick_;
	bool ready_to_place_;

	// Parameters
	double max_ac_execution_time;
	double slope_max;

	// Variables
	std::string ns_;
	haf_grasping::GraspInput grasp_req_;
	haf_grasping::GraspOutput grasp_res_;
	haf_grasping::CalcGraspPointsServerActionGoal goal_;
	geometry_msgs::Pose place_pose_;
	geometry_msgs::Pose place_pose_soft;
	geometry_msgs::Pose place_pose_semi_soft;
	geometry_msgs::Pose place_pose_hard;
	geometry_msgs::Pose moveto_pose_;

	int eval_thresh;
	double last_slope;

	// ROS
	actionlib::SimpleActionClient <haf_grasping::CalcGraspPointsServerAction>
	haf_ac_;
	actionlib::SimpleActionClient <pr2_picknplace_msgs::PickPlaceAction>
	pickplace_ac_;
	ros::Subscriber point_cloud_sub_;
	ros::ServiceClient request_pc_;
	ros::ServiceClient new_grasp_start_client;
	ros::ServiceClient get_slope_client;
	std_srvs::Empty empty_;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
};

#endif  /* GRASP_MANAGER_HPP */
