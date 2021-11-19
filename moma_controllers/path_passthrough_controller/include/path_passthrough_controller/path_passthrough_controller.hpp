//
// Created by Julian Keller on 04.11.21 based on admittance controller.
//

#pragma once

#include <nav_msgs/Path.h>
#include <mutex>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

namespace moma_controllers {

class PathPassthroughController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
 public:
  PathPassthroughController() = default;
  ~PathPassthroughController() = default;

  bool init(hardware_interface::JointStateInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration&) override;
  
  void starting(const ros::Time& time) override {
      path_received_ = false;
      ROS_INFO("[PathPassthroughController] Starting.");
  };
  void stopping(const ros::Time& /*time*/) override {
    ROS_INFO("[PathPassthroughController] Stopping.");
  };

 private:
  void path_callback(const nav_msgs::PathConstPtr& msg);

 private:
  bool verbose_;
  std::atomic_bool path_received_;

  // ROS
  ros::Subscriber path_subscriber_;
  ros::Publisher desiredPathPublisher_;
  nav_msgs::Path desiredPath_;
  nav_msgs::Path receivedPath_;
  std::mutex pathMutex_;
};

}  // namespace moma_controllers
