//
// Created by giuseppe on 24.01.21.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/Path.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <realtime_tools/realtime_publisher.h>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>


namespace moma_controllers {

class PathAdmittanceController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
 public:
  PathAdmittanceController();
  ~PathAdmittanceController() = default;

  bool init(hardware_interface::JointStateInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;
  
  void starting(const ros::Time& time) override{};
  void stopping(const ros::Time& /*time*/) override {};

 private:
  void adjustPath(nav_msgs::Path& desiredPath);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);
  void path_callback(const nav_msgs::PathConstPtr& msg);

  /**
   * Utility function to parse a vector of 3 gains
   * @param nh: ros node handle
   * @param name: name of the gains on the parameter server
   * @param gains: the vector object
   */
  template <int N>
  bool parse_vector(ros::NodeHandle& nh, const std::string& name,
                    Eigen::Matrix<double, N, 1>& gains);

  void threshold(Eigen::Vector3d& v, const Eigen::Vector3d& tau);

 private:
  std::atomic_bool wrench_received_;

  // ROS
  std::unique_ptr<ros::CallbackQueue> wrench_callback_queue_;
  ros::Subscriber wrench_subscriber_;
  ros::Subscriber path_subscriber_;
  ros::Publisher desiredPathPublisher_;
  nav_msgs::Path desiredPath_;
  nav_msgs::Path receivedPath_;
  std::mutex pathMutex_;

  std::mutex wrench_mutex_;
  geometry_msgs::WrenchStamped wrench_;

  // Gains
  Eigen::Vector3d Kp_linear_;
  Eigen::Vector3d Kp_angular_;
  Eigen::Vector3d Ki_linear_;
  Eigen::Vector3d Ki_angular_;

  // Errors
  double last_time_;
  Eigen::Vector3d force_error_;
  Eigen::Vector3d force_integral_;
  Eigen::Vector3d torque_error_;
  Eigen::Vector3d torque_integral_;
  Eigen::Vector3d force_integral_max_;
  Eigen::Vector3d torque_integral_max_;

  Eigen::Vector3d force_threshold_;
  Eigen::Vector3d torque_threshold_;

  // TF
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
};

}  // namespace moma_controllers
