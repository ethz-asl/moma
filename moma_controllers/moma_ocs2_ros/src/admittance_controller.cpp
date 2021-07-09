//
// Created by giuseppe on 24.01.21.
//

#include "moma_ocs2_ros/admittance_controller.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace moma_controllers;

AdmittanceController::AdmittanceController(ros::NodeHandle& nh)
    : tf_listener_(tf_buffer_) {
  std::string wrench_topic;
  if (!nh.param<std::string>("/admittance_controller/wrench_topic", wrench_topic, "/wrench")) {
    ROS_WARN_STREAM("Failed to parse wrench topic, defaulting to /wrench");
  }

  parse_vector<3>(nh, "/admittance_controller/kp_linear_gains", Kp_linear_);
  parse_vector<3>(nh, "/admittance_controller/kp_angular_gains", Kp_angular_);
  parse_vector<3>(nh, "/admittance_controller/ki_linear_gains", Ki_linear_);
  parse_vector<3>(nh, "/admittance_controller/ki_angular_gains", Ki_angular_);
  parse_vector<3>(nh, "/admittance_controller/force_integral_max", force_integral_max_);
  parse_vector<3>(nh, "/admittance_controller/torque_integral_max", torque_integral_max_);
  
  wrench_callback_queue_ = std::unique_ptr<ros::CallbackQueue>(new ros::CallbackQueue());
  ros::SubscribeOptions so;
  so.init<geometry_msgs::WrenchStamped>(
      wrench_topic, 1, boost::bind(&AdmittanceController::wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  wrench_subscriber_ = nh.subscribe(so);

  force_integral_.setZero();
  torque_integral_.setZero();
  last_time_ = -1;
  wrench_received_ = false;

  // parse thresholds
  parse_vector<3>(nh, "/admittance_controller/force_threshold", force_threshold_);
  parse_vector<3>(nh, "/admittance_controller/torque_threshold", torque_threshold_);
  for (size_t i=0; i<3; i++){
    if (force_threshold_(i) < 0){
      ROS_WARN("Negative force threshold: setting to 0");
      force_threshold_(i) = 0.0;
    }
    if (torque_threshold_(i) < 0){
      ROS_WARN("Negative torque threshold: setting to 0");
      force_threshold_(i) = 0.0;
    }
  } 
}

void AdmittanceController::adjustPath(nav_msgs::Path& desiredPath) {
  wrench_callback_queue_->callAvailable();
  if (!wrench_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "No wrench received. Not modifying the path.");
    return;
  }

  // Update time interval for integral computation
  double dt;
  if (last_time_ < 0) {
    dt = 0;
    last_time_ = ros::Time::now().toSec();
  } else {
    dt = ros::Time::now().toSec() - last_time_;
    last_time_ = ros::Time::now().toSec();
  }

  // Update sensor pose in a base frame
  geometry_msgs::TransformStamped transform;
  try {
    // target_frame, source_frame ...
    transform =
        tf_buffer_.lookupTransform(desiredPath.header.frame_id, wrench_.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
    return;
  }
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix3d R(q);

  // Update measured wrench and tracking errors
  force_error_ = Eigen::Vector3d(wrench_.wrench.force.x, 
                                 wrench_.wrench.force.y, 
                                 wrench_.wrench.force.z);
  threshold(force_error_, force_threshold_);

  force_integral_ += force_error_ * dt;
  force_integral_ = force_integral_.cwiseMax(-force_integral_max_);
  force_integral_ = force_integral_.cwiseMin(force_integral_max_);
  Eigen::Vector3d delta_position =
      Kp_linear_.cwiseProduct(force_error_) + Ki_linear_.cwiseProduct(force_integral_);
  Eigen::Vector3d delta_position_transformed = R * delta_position;

  torque_error_ = Eigen::Vector3d(wrench_.wrench.torque.x, 
                                  wrench_.wrench.torque.y, 
                                  wrench_.wrench.torque.z);
  threshold(torque_error_, torque_threshold_);

  torque_integral_ += torque_error_ * dt;
  torque_integral_ = torque_integral_.cwiseMax(-torque_integral_max_);
  torque_integral_ = torque_integral_.cwiseMin(torque_integral_max_);
  Eigen::Vector3d rotationImg =
      Kp_angular_.cwiseProduct(torque_error_) + Ki_angular_.cwiseProduct(torque_integral_);
  Eigen::Quaterniond delta_rotation(1, 0, 0, 0);
  double r = rotationImg.norm();
  if (r > 0) {
    delta_rotation.w() = std::cos(r);
    Eigen::Vector3d quatImg = R * (std::sin(r) / r * rotationImg);
    delta_rotation.x() = quatImg[0];
    delta_rotation.y() = quatImg[1];
    delta_rotation.z() = quatImg[2];
  }


  ROS_DEBUG_STREAM_THROTTLE(
      2.0, "Adjusting path: " << std::endl
                              << "delta position (sensor frame) = " << delta_position.transpose()
                              << std::endl
                              << "delta position (base frame) = "
                              << delta_position_transformed.transpose()
                              << "delta rotation norm (deg): = " << r * 180.0 / M_PI);
  for (auto& pose : desiredPath.poses) {
    pose.pose.position.x += delta_position_transformed.x();
    pose.pose.position.y += delta_position_transformed.y();
    pose.pose.position.z += delta_position_transformed.z();
    Eigen::Quaterniond new_orientation(pose.pose.orientation.w, pose.pose.orientation.x,
                                       pose.pose.orientation.y, pose.pose.orientation.z);
    new_orientation =  delta_rotation * new_orientation;
    pose.pose.orientation.x = new_orientation.x();
    pose.pose.orientation.y = new_orientation.y();
    pose.pose.orientation.z = new_orientation.z();
    pose.pose.orientation.w = new_orientation.w();
  }
}

void AdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_ = *msg;
  wrench_received_ = true;
}

template <int N>
void AdmittanceController::parse_vector(ros::NodeHandle& nh, const std::string& name,
                                            Eigen::Matrix<double, N, 1>& gains) {
  std::vector<double> gains_vector;
  if (!nh.param<std::vector<double>>(name, gains_vector, {})) {
    ROS_WARN_STREAM("Failed to parse " << name << ", defaulting to zero vector");
  }
  if (gains_vector.size() != N) {
    ROS_ERROR_STREAM("Gains have the wrong size! Defaulting to zero");
    gains_vector.clear();
    gains_vector.resize(N, 0.0);
  }

  for (size_t i = 0; i < N; i++) gains(i) = gains_vector[i];
}

void AdmittanceController::threshold(Eigen::Vector3d& v, const Eigen::Vector3d& tau){
  for (size_t i=0; i<v.size(); i++){
    if (v(i) == 0) continue;
    v(i) = (std::abs(v(i)) - tau(i)) > 0 ? v(i) - v(i)/std::abs(v(i)) * tau(i) : 0.0; 
  }
}
