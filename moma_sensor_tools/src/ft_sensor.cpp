//
// Created by giuseppe on 25.01.21.
//

#include "moma_sensor_tools/ft_sensor.h"

namespace moma_sensor_tools{

bool ForceTorqueSensor::reset() {
  if (!get_ft_calibration_from_file(calibration_file_, calibration_data_)) {
    ROS_ERROR_STREAM("Failed to reset FT Sensor calibration.");
    return false;
  }
  ROS_INFO_STREAM(calibration_data_);
  wrench_received_ = false;
  return true;
}

bool ForceTorqueSensor::init() {
  if (!nh_.param<std::string>("sensor_frame", sensor_frame_, "")) {
    ROS_ERROR_STREAM("Failed to parse sensor_frame");
    return false;
  }

  if (!nh_.param<std::string>("gravity_aligned_frame", gravity_aligned_frame_, "")) {
    ROS_ERROR_STREAM("Failed to parse gravity_aligned_frame");
    return false;
  }

  //std::string imu_topic;
  //if (!nh_.param<std::string>("imu_topic", imu_topic, "")) {
  //  ROS_ERROR_STREAM("Failed to parse imu_topic");
  //  return false;
  //}

  bool estimate_bias_at_startup;
  if (!nh_.param<bool>("estimate_bias_at_startup", estimate_bias_at_startup, true)) {
    ROS_ERROR_STREAM("Failed to parse estimate_bias_at_startup");
    return false;
  }

  if (!nh_.param<std::string>("calibration_file", calibration_file_, "")) {
    ROS_ERROR_STREAM("Could not find calibration_file on param server.");
    return false;
  }
  if (!get_ft_calibration_from_file(calibration_file_, calibration_data_)) {
    ROS_ERROR_STREAM("Failed to parse FT Sensor calibration.");
    return false;
  }
  ROS_INFO_STREAM(calibration_data_);

  std::string raw_wrench_topic;
  if (!nh_.param<std::string>("raw_wrench_topic", raw_wrench_topic, "")) {
    ROS_ERROR_STREAM("Failed to parse raw_wrench_topic");
    return false;
  }

  std::string out_wrench_topic;
  if (!nh_.param<std::string>("out_wrench_topic", out_wrench_topic, "")) {
    ROS_ERROR_STREAM("Failed to parse out_wrench_topic");
    return false;
  }
  wrench_publisher_ = nh_.advertise<geometry_msgs::WrenchStamped>(out_wrench_topic, 1);

  ros::SubscribeOptions so;
  wrench_callback_queue_ = std::make_unique<ros::CallbackQueue>();
  so.init<geometry_msgs::WrenchStamped>(
      raw_wrench_topic, 1, boost::bind(&ForceTorqueSensor::raw_wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  raw_wrench_subscriber_ = nh_.subscribe(so);
  wrench_received_ = false;

  estimate_bias_measurements_ = 0;
  estimate_bias_ = (estimate_bias_at_startup) ? true : false;
  estimate_bias_service_ = nh_.advertiseService("/estimate_bias", &ForceTorqueSensor::estimate_bias_callback, this);
  
  //imu_received_ = false;
  //imu_subscriber_ = nh_.subscribe(imu_topic, 1, &ForceTorqueSensor::imu_callback, this);

  nh_.param<double>("filter_constant", alpha_, 1.0);
  if (alpha_ < 0 || alpha_ > 1){
    ROS_WARN("filter_constant must be between 0 and 1. No filtering (filter_constant = 1)");
  }
  wrench_compensated_filtered_.setZero();
  
  nh_.param<bool>("debug", debug_, false);
  tool_wrench_publisher_ = nh_.advertise<geometry_msgs::WrenchStamped>("/tool_wrench", 1);
  tool_wrench_ros_.header.frame_id = sensor_frame_;
  return true;
}

void ForceTorqueSensor::update() {
  wrench_callback_queue_->callAvailable();
  if (!wrench_received_){
    ROS_WARN_STREAM_THROTTLE(1.0, "No wrench message received yet.");
    return ;
  }

  //if (!imu_received_){
  //  ROS_WARN_STREAM_THROTTLE(1.0, "No imu message received yet.");
  //  return ;
 // }

  // method a: get local gravity vector using a gravity aligned frame
  // In fixed frame gravity is aligned with -z axis
  geometry_msgs::TransformStamped transform;
  try {
     // target_frame, source_frame ...
     transform = tf2_buffer_.lookupTransform(sensor_frame_, gravity_aligned_frame_, ros::Time(0));
  } catch (tf2::TransformException& ex) {
     ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
     return;
  }
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d gravity = R * Eigen::Vector3d::UnitZ() * -9.81;
  tool_wrench_.get_force() = gravity * calibration_data_.mass;
  tool_wrench_.get_torque() = calibration_data_.com.cross(tool_wrench_.get_force());
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Wrench tool" << tool_wrench_);

  if (debug_){
    tool_wrench_ros_.wrench.force.x = tool_wrench_.get_force().x();
    tool_wrench_ros_.wrench.force.y = tool_wrench_.get_force().y();
    tool_wrench_ros_.wrench.force.z = tool_wrench_.get_force().z(); 
    tool_wrench_ros_.wrench.torque.x = tool_wrench_.get_torque().x();
    tool_wrench_ros_.wrench.torque.y = tool_wrench_.get_torque().y();
    tool_wrench_ros_.wrench.torque.z = tool_wrench_.get_torque().z();
    tool_wrench_ros_.header.stamp = ros::Time::now();
    tool_wrench_publisher_.publish(tool_wrench_ros_);
  }
  /// alternative using IMU
  /*
  Eigen::Vector3d gravity_temp(imu_.linear_acceleration.x, 
                               imu_.linear_acceleration.y, 
                               imu_.linear_acceleration.z);
  geometry_msgs::TransformStamped transform;
  try {
    // target_frame, source_frame ...
    transform = tf2_buffer_.lookupTransform(sensor_frame_, imu_.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    return;
  }
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  
  Eigen::Matrix3d R(q);
  Eigen::Vector3d gravity = R * gravity_temp;

  tool_wrench_.get_force() = gravity * calibration_data_.mass;
  tool_wrench_.get_torque() = calibration_data_.com.cross(tool_wrench_.get_force());
  */

  Eigen::Matrix<double, 6, 1> raw_wrench;
  tf::wrenchMsgToEigen(wrench_raw_.wrench, raw_wrench);

  Eigen::Matrix<double, 6, 1> compensated_wrench = raw_wrench;
  compensated_wrench -= tool_wrench_.to_vector();
  
  // Use payload compensated wrench to estimate bias
  if (estimate_bias_){
    bias_ += compensated_wrench;
    estimate_bias_measurements_++;
    if (estimate_bias_measurements_ > 1000){
      bias_ = 0.001 * bias_;
      calibration_data_.bias = bias_;
      estimate_bias_ = false;
      ROS_INFO_STREAM("Estimated bias is: " << calibration_data_.bias.transpose() << std::endl
        << "   current tool wrench is: " << tool_wrench_.to_vector().transpose() << std::endl
        << "   current raw is: " << raw_wrench.transpose() << std::endl
        << "   gravity in sensor frame is: " << gravity.transpose());
    }
  }

  // remove bias
  compensated_wrench -= calibration_data_.get_bias();

  // filter
  wrench_compensated_filtered_ = (1-alpha_) * wrench_compensated_filtered_ +  alpha_ * compensated_wrench;
  tf::wrenchEigenToMsg(wrench_compensated_filtered_, wrench_compensated_.wrench);
  wrench_compensated_.header = wrench_raw_.header;

  wrench_publisher_.publish(wrench_compensated_);
  
}

void ForceTorqueSensor::imu_callback(const sensor_msgs::ImuConstPtr& msg){
  imu_ = *msg;
  imu_received_ = true;
}

void ForceTorqueSensor::raw_wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  wrench_raw_ = *msg;
  wrench_received_ = true;
}

bool ForceTorqueSensor::estimate_bias_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse& ){
  if (estimate_bias_){
    ROS_WARN("Already estimating the bias");
    return true;
  }
  bias_.setZero();
  estimate_bias_measurements_ = 0;
  estimate_bias_ = true;
  return true;
}
}