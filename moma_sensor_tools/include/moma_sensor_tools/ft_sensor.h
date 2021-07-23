//
// Created by giuseppe on 25.01.21.
//

#pragma once
#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <moma_sensor_tools/ft_sensor_utils.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

/// Subscribe to the raw data and removes the payload wrench
/// and sensor bias
namespace moma_sensor_tools {

class ForceTorqueSensor {
 public:
  ForceTorqueSensor() = delete;
  explicit ForceTorqueSensor(ros::NodeHandle& nh) : nh_(nh), tf2_listener_(tf2_buffer_) {}
  ~ForceTorqueSensor() = default;

 public:
  bool reset();
  bool init();
  void update();

 private:
  bool estimate_bias_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse& );

 private:
  void imu_callback(const sensor_msgs::ImuConstPtr& msg);
  void raw_wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<ros::CallbackQueue> wrench_callback_queue_;
  ros::Subscriber raw_wrench_subscriber_;
  ros::Publisher wrench_publisher_;


  int estimate_bias_measurements_;
  std::atomic_bool estimate_bias_;
  Eigen::Matrix<double, 6, 1> bias_;
  ros::ServiceServer estimate_bias_service_;

  std::string calibration_file_;
  Wrench tool_wrench_;

  FTSensorCalibrationData calibration_data_;
  geometry_msgs::WrenchStamped wrench_raw_;
  geometry_msgs::WrenchStamped wrench_compensated_;

  // TF
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::Buffer tf2_buffer_;

  std::string gravity_aligned_frame_;
  std::string sensor_frame_;

  bool wrench_received_;

  double alpha_;
  Eigen::Matrix<double, 6, 1> wrench_compensated_filtered_;

  sensor_msgs::Imu imu_;
  bool imu_received_;
  ros::Subscriber imu_subscriber_;

  bool debug_;
  geometry_msgs::WrenchStamped tool_wrench_ros_;
  ros::Publisher tool_wrench_publisher_;
};

}  // namespace sensor_tools::ft
