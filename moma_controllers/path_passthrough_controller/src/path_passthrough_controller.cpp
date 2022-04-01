//
// Created by Julian Keller on 04.11.21 based on admittance controller.
//

#include "path_passthrough_controller/path_passthrough_controller.hpp"

#include <pluginlib/class_list_macros.h>


bool PathPassthroughController::init(hardware_interface::JointStateInterface* hw,
                                     ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
std::string path_in_topic;
if (!controller_nh.param<std::string>("path_in_topic", path_in_topic, "/demo_path")) {
             ROS_WARN_STREAM("[PathPassthroughController] Failed to parse path_in_topic topic");
    return false;
  }
  path_subscriber_ =
      controller_nh.subscribe(path_in_topic, 1, &PathPassthroughController::path_callback, this);
  ROS_INFO_STREAM("[PathPassthroughController] Subscribing to path topic [" << path_in_topic
                                                                            << "]");

  std::string path_out_topic;
  if (!controller_nh.param<std::string>("path_out_topic", path_out_topic, "/demo_path")) {
    ROS_WARN_STREAM("[PathPassthroughController] Failed to parse path_out_topic topic");
    return false;
  }
  desiredPathPublisher_ = controller_nh.advertise<nav_msgs::Path>(path_out_topic, 1);
  ROS_INFO_STREAM("[PathPassthroughController] Publishing to path topic [" << path_out_topic
                                                                           << "]");

  ROS_INFO("[PathPassthroughController] Initialized!");
  return true;
}

void PathPassthroughController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  if (!path_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "No path received yet.");
    return;
  }

  {
    std::unique_lock<std::mutex> lock(pathMutex_);
    desiredPath_ = receivedPath_;
  }

  desiredPathPublisher_.publish(desiredPath_);
}

void PathPassthroughController::path_callback(const nav_msgs::PathConstPtr& msg) {
  std::unique_lock<std::mutex> lock(pathMutex_);
  receivedPath_ = *msg;
  path_received_ = true;
}

}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::PathPassthroughController,
                       controller_interface::ControllerBase)
