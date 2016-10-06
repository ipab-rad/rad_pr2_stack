#include <ros/ros.h>
#include <sstream>

int main(int argc, char** argv) {
	ros::init(argc, argv, "pr2_eye_track_demo");
	ros::NodeHandle n;

	ros::Rate r(30);
	while (ros::ok()) {
		ROS_INFO("Waiting!");
		ros::spinOnce();

		r.sleep();
	}
}
