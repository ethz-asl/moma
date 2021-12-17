//
// Created by giuseppe on 31.12.20.
//

#pragma once

#include <mutex>

#include <moma_ocs2/definitions.h>
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
  using state_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::STATE_DIM, 1>;
  using input_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::INPUT_DIM, 1>;
  using joint_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::ARM_INPUT_DIM, 1>;
  using base_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::BASE_INPUT_DIM, 1>;

  MpcController() = delete;
  explicit MpcController(const ros::NodeHandle& nh);
  ~MpcController();

  bool init();
  void start(const state_vector_t& initial_observation);
  void update(const ros::Time& time, const state_vector_t& observation);
  void stop();
  /**
   * Updates the desired cost trajectories from path message.
   * @param desiredPath
   */
  void pathCallback(const nav_msgs::PathConstPtr& desiredPath);
  inline state_vector_t getPositionCommand() const { return positionCommand_; }
  inline input_vector_t getVelocityCommand() const { return velocityCommand_; }
  /*inline joint_vector_t getArmPositionCommand() const { return positionCommand_.tail<ocs2::mobile_manipulator::ARM_INPUT_DIM>(); }
  inline joint_vector_t getArmVelocityCommand() const { return velocityCommand_.tail<ocs2::mobile_manipulator::ARM_INPUT_DIM>(); }
  inline base_vector_t getBasePositionCommand() const { return positionCommand_.head<ocs2::mobile_manipulator::BASE_INPUT_DIM>(); }
  inline base_vector_t getBaseVelocityCommand() const { return velocityCommand_.head<ocs2::mobile_manipulator::BASE_INPUT_DIM>(); }*/
  inline const ros::NodeHandle& getNodeHandle() { return nh_; }
  inline std::string getDescription() { return robot_description_; }

 protected:

  /**
   * Optional preprocessing step for the tracked path.
   * @param desiredPath
   */
  virtual void adjustPath(nav_msgs::Path& desiredPath) {};

 private:
  void advanceMpc();
  void setObservation(const state_vector_t& q);
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

  state_vector_t initialState_;
  state_vector_t positionCommand_;
  input_vector_t velocityCommand_;
  std::vector<std::string> jointNames_;

  ros::NodeHandle nh_;
  ros::Publisher observationPublisher_;
  ros::Publisher commandPublisher_;
  ros::Subscriber targetPathSubscriber_;

 private:
  std::atomic_bool stopped_;
  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;
  double mpcFrequency_;

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
  realtime_tools::RealtimePublisher<nav_msgs::Path> rollout_publisher_;

};  
}  // namespace mobile_manipulator