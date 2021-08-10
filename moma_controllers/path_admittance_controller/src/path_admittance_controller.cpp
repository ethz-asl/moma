//
// Created by giuseppe on 24.01.21.
//

#include "path_admittance_controller/path_admittance_controller.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_list_macros.h>

namespace moma_controllers{

PathAdmittanceController::PathAdmittanceController(){
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
}


bool PathAdmittanceController::init(hardware_interface::JointStateInterface* hw,
                                    ros::NodeHandle& root_nh,
                                    ros::NodeHandle& controller_nh){
  std::string wrench_topic;
  if (!controller_nh.param<std::string>("wrench_topic", wrench_topic, "/wrench")) {
    ROS_WARN_STREAM("[PathAdmittanceController] Failed to parse wrench topic");
    return false;
  }
  ROS_INFO_STREAM("[PathAdmittanceController] Subscribing to wrench topic [" << wrench_topic << "]");


  std::string path_in_topic;
  if (!controller_nh.param<std::string>("path_in_topic", path_in_topic, "/demo_path")) {
    ROS_WARN_STREAM("[PathAdmittanceController] Failed to parse path_in_topic topic");
    return false;
  }
  path_subscriber_ = controller_nh.subscribe(path_in_topic, 1, &PathAdmittanceController::path_callback, this);
  ROS_INFO_STREAM("[PathAdmittanceController] Subscribing to path topic [" << path_in_topic << "]");
  
  std::string path_out_topic;
  if (!controller_nh.param<std::string>("path_out_topic", path_out_topic, "/demo_path")) {
    ROS_WARN_STREAM("[PathAdmittanceController] Failed to parse path_out_topic topic");
    return false;
  }
  desiredPathPublisher_ = controller_nh.advertise<nav_msgs::Path>(path_out_topic, 1);
  ROS_INFO_STREAM("[PathAdmittanceController] Publishing to path topic [" << path_out_topic << "]");
  
  bool ok = true;
  ok &= parse_vector<3>(controller_nh, "kp_linear_gains", Kp_linear_);
  ok &= parse_vector<3>(controller_nh, "kp_angular_gains", Kp_angular_);
  ok &= parse_vector<3>(controller_nh, "ki_linear_gains", Ki_linear_);
  ok &= parse_vector<3>(controller_nh, "ki_angular_gains", Ki_angular_);
  ok &= parse_vector<3>(controller_nh, "force_integral_max", force_integral_max_);
  ok &= parse_vector<3>(controller_nh, "torque_integral_max", torque_integral_max_);
  ok &= parse_vector<3>(controller_nh, "force_threshold", force_threshold_);
  ok &= parse_vector<3>(controller_nh, "torque_threshold", torque_threshold_);

  if(!ok){
    ROS_ERROR("[PathAdmittanceController] Failed to parse params.");
    return false;
  }

  wrench_callback_queue_ = std::unique_ptr<ros::CallbackQueue>(new ros::CallbackQueue());
  ros::SubscribeOptions so;
  so.init<geometry_msgs::WrenchStamped>(
      wrench_topic, 1, boost::bind(&PathAdmittanceController::wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  wrench_subscriber_ = controller_nh.subscribe(so);
  ROS_INFO("[PathAdmittanceController] Created wrench callback.");

  force_integral_.setZero();
  torque_integral_.setZero();
  last_time_ = -1;
  wrench_received_ = false;

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
  ROS_INFO("[PathAdmittanceController] Initialized!");
  return true;
}

void PathAdmittanceController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  if (!path_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "No path received yet.");
    return;
  }

  {
    std::unique_lock<std::mutex> lock(pathMutex_);
    desiredPath_ = receivedPath_;
  }

  wrench_callback_queue_->callAvailable();
  if (!wrench_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "No wrench received. Not modifying the path.");
    desiredPathPublisher_.publish(desiredPath_);
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
        tf_buffer_.lookupTransform(desiredPath_.header.frame_id, wrench_.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(2.0, std::string{ex.what()} + ", publishing current path");
    desiredPathPublisher_.publish(desiredPath_);
    return;
  }
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix3d R(q);

  // Update measured wrench and tracking errors
  force_ext_ = Eigen::Vector3d(wrench_.wrench.force.x,
                                 wrench_.wrench.force.y, 
                                 wrench_.wrench.force.z);
  threshold(force_ext_, force_threshold_);

  force_integral_ += force_ext_ * dt;
  force_integral_ = force_integral_.cwiseMax(-force_integral_max_);
  force_integral_ = force_integral_.cwiseMin(force_integral_max_);
  Eigen::Vector3d delta_position =
      Kp_linear_.cwiseProduct(force_ext_) + Ki_linear_.cwiseProduct(force_integral_);
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
  for (auto& pose : desiredPath_.poses) {
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
  desiredPathPublisher_.publish(desiredPath_);
}

void PathAdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_ = *msg;
  wrench_received_ = true;
}

void PathAdmittanceController::path_callback(const nav_msgs::PathConstPtr& msg){
  std::cout << "Path received!" << std::endl;
  std::unique_lock<std::mutex> lock(pathMutex_);
  receivedPath_ = *msg;
  path_received_ = true;
}

template <int N>
bool PathAdmittanceController::parse_vector(ros::NodeHandle& nh, const std::string& name,
                                            Eigen::Matrix<double, N, 1>& gains) {
  std::vector<double> gains_vector;
  if (!nh.param<std::vector<double>>(name, gains_vector, {})) {
    ROS_WARN_STREAM("Failed to parse " << name);
    return false;
  }
  if (gains_vector.size() != N) {
    ROS_ERROR_STREAM("Gains have the wrong size! Defaulting to zero");
    gains_vector.clear();
    gains_vector.resize(N, 0.0);
  }

  for (size_t i = 0; i < N; i++) gains(i) = gains_vector[i];
  return true;
}

void PathAdmittanceController::threshold(Eigen::Vector3d& v, const Eigen::Vector3d& tau){
  for (int i=0; i<v.size(); i++){
    if (v(i) == 0) continue;
    v(i) = (std::abs(v(i)) - tau(i)) > 0 ? v(i) - v(i)/std::abs(v(i)) * tau(i) : 0.0;
  }
}

}  // namespace moma_controllers

PLUGINLIB_EXPORT_CLASS(moma_controllers::PathAdmittanceController,
                       controller_interface::ControllerBase)