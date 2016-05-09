/**
 * @file      pr2_localise_table.cpp
 * @brief
 * @author    Daniel Angelov <d.angelov@ed.ac.uk>
 * @date      2016-02-15
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include <ros/ros.h>
#include <polled_camera/GetPolledImage.h>

void update_prosilica_mode(std::string mode) {
  // The two main modes are 'polled' & 'streaming'
  std::stringstream cmd;
  cmd << "rosrun dynamic_reconfigure dynparam set ";
  cmd << "/prosilica_driver trigger_mode " << mode;
  system(cmd.str().c_str());
  ROS_INFO_STREAM("Setting prosilica mode: " << mode);
  ros::WallDuration(0.5).sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_localise_table");
  ros::NodeHandle nh("~");

  update_prosilica_mode("polled");

  ros::ServiceClient req_image =
    nh.serviceClient<polled_camera::GetPolledImage>("/prosilica/request_image");
  polled_camera::GetPolledImage srv;

  std::string cam_ns;
  if (!nh.getParam("camera_ns", cam_ns)) {
    ROS_WARN("[PR2_LOCALISE_TABLE] Parameters were not loaded!.");
  }
  srv.request.response_namespace = cam_ns;
  srv.request.timeout = ros::Duration(30.0);

  ROS_INFO_STREAM(srv.request.response_namespace);

  ros::Rate r(30);
  while (ros::ok()) {
    ROS_INFO("[PR2_LOCALISE_TABLE] TODO: Stream tf");
    if (req_image.call(srv)) {
      ROS_INFO_STREAM("[PR2_LOCALISE_TABLE] Request: " << ((srv.response.success) ?
                                                           "Success" : "Failed"));
    } else {
      ROS_ERROR("[PR2_LOCALISE_TABLE] Failed to call service `request_image`!\n"
                "Please restart!");
      continue; //return 1;
    }
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
