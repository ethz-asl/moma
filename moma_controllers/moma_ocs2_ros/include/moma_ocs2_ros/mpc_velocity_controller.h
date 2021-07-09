//
// Created by giuseppe on 31.12.20.
//

#pragma once

#include <mutex>

#include <moma_ocs2/MobileManipulatorInterface.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace moma_controllers {

class MpcController{
 public:
  using joint_vector_t = Eigen::Matrix<double, 7, 1>;

  MpcController() = delete;
  explicit MpcController(const ros::NodeHandle& nh);
  ~MpcController();

  bool init();
  void start(const joint_vector_t& initial_observation);
  void update(const ros::Time& time, const joint_vector_t& observation);
  void stop();
  /**
   * Updates the desired cost trajectories from path message.
   * @param desiredPath
   */
  void pathCallback(const nav_msgs::PathConstPtr& desiredPath);
  inline Eigen::VectorXd get_position_command() const { return positionCommand_; }
  inline Eigen::VectorXd get_velocity_command() const { return velocityCommand_; }
  inline const ros::NodeHandle& get_node_handle() { return nh_; }
  inline std::string get_description() { return robot_description_; }

 protected:

  /**
   * Optional preprocessing step for the tracked path.
   * @param desiredPath
   */
  virtual void adjustPath(nav_msgs::Path& desiredPath) {};

 private:
  void advanceMpc();
  void setObservation(const joint_vector_t& q);
  void updateCommand();
  void writeDesiredPath(const nav_msgs::Path& path);
  void publishObservation();

  // path processing
  static bool sanityCheck(const nav_msgs::Path& path);
  void transformPath(nav_msgs::Path& desiredPath) ;
  void adjustPathTime(nav_msgs::Path& desiredPath) const;

  // publish ros
  void publishRos();
  void publishCurrentRollout();
  void publishDesiredPath();

 protected:
  double start_time_;

  std::string base_link_;
  std::string tool_link_;
  std::string robot_description_;

  joint_vector_t jointInitialState_;
  joint_vector_t positionCommand_;
  joint_vector_t velocityCommand_;
  std::vector<std::string> jointNames_;

  ros::NodeHandle nh_;
  ros::Publisher observationPublisher_;
  ros::Publisher commandPublisher_;
  ros::Subscriber targetPathSubscriber_;

 private:
  std::atomic_bool stopped_;
  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;
  double mpcFrequency_;
  std::string taskFile_;

  std::atomic_bool policyReady_;
  std::atomic_bool referenceEverReceived_;
  std::atomic_bool observationEverReceived_;

  std::thread mpcThread_;
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  std::mutex observationMutex_;
  ocs2::SystemObservation observation_;

  std::mutex desiredPathMutex_;
  nav_msgs::Path desiredPath_;

  std::mutex policyMutex_;
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> mm_interface_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Eigen::Affine3d T_tool_ee_;   // transform from frame tracked by MPC to the actual tool frame
  Eigen::Affine3d T_base_x_;    // transform from reference frame incoming trajectory and arm base
  Eigen::Affine3d T_x_tool_;    // transfrom from point in the path to the path reference frame
  Eigen::Affine3d T_base_ee_;   // transform from base to desired end effector pose

  // realtime publisher
  double publishRosFrequency_;
  std::thread publishRosThread_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> command_path_publisher_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> desired_path_publisher_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> rollout_publisher_;

};  
}  // namespace mobile_manipulator