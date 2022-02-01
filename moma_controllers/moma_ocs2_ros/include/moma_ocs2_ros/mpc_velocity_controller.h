//
// Created by giuseppe on 31.12.20.
//

#pragma once

#include <mutex>

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <robot_control/modeling/robot_wrapper.h>

#include <moma_ocs2/definitions.h>
#include <moma_ocs2/MobileManipulatorInterface.h>
#include <moma_ocs2/definitions.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace moma_controllers {

template <size_t ArmInputDim>
class MpcController {
 public:
  using state_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::STATE_DIM(ArmInputDim), 1>;
  using input_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::INPUT_DIM(ArmInputDim), 1>;
  using joint_vector_t = Eigen::Matrix<double, ArmInputDim, 1>;
  using base_vector_t = Eigen::Matrix<double, ocs2::mobile_manipulator::BASE_INPUT_DIM, 1>;

  MpcController() = delete;
  explicit MpcController(const ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {}
  ~MpcController() {
    ROS_INFO("[MpcController::~MpcController] destroying mpc controller.");
    unloaded_ = true;
    if (mpcThread_.joinable()){
      mpcThread_.join();
    }
  }

  bool init() {
    std::string robotName;

    // Params
    if (!nh_.param("/ocs2_mpc/task_file", taskFile_, {})){
      ROS_ERROR("Failed to retrieve /ocs2_mpc/task_file from param server.");
      return 0;
    }
    if (!nh_.param("/ocs2_mpc/robot_description_ocs2", urdfXML_, {})){
      ROS_ERROR("Failed to retrieve /ocs2_mpc/robot_description_ocs2 from param server.");
      return 0;
    }
    int baseTypeInt;
    if (!nh_.param("/ocs2_mpc/base_type", baseTypeInt, 0)){
      ROS_ERROR("Failed to retrieve /ocs2_mpc/base_type from param server.");
      return 0;
    }
    if (baseTypeInt >= ocs2::mobile_manipulator::BASE_TYPE_COUNT){
      ROS_ERROR("The value of base_type is not supported.");
      return 0;
    }
    baseType_ = static_cast<ocs2::mobile_manipulator::BaseType>(baseTypeInt);

    mm_interface_.reset(
        new ocs2::mobile_manipulator::MobileManipulatorInterface(taskFile_, urdfXML_, ArmInputDim, baseType_));
    mpcPtr_ = mm_interface_->getMpc();

    const std::unique_ptr<rc::RobotWrapper> robot_model = std::unique_ptr<rc::RobotWrapper>(new rc::RobotWrapper());
    robot_model->initFromXml(urdfXML_);
    if (ocs2::mobile_manipulator::INPUT_DIM(ArmInputDim) != robot_model->getDof()) {
      ROS_ERROR_STREAM("The arm input dimension size is not correct. Expected " << ocs2::mobile_manipulator::INPUT_DIM(ArmInputDim)
                                                                                << " but got " << robot_model->getDof());
      return 0;
    }

    nh_.param<std::string>("tool_link", tool_link_, "tool_frame");
    nh_.param<double>("mpc_frequency", mpcFrequency_, 1e3);
    nh_.param<double>("publish_ros_frequency", publishRosFrequency_, 20);

    std::string commandTopic;
    nh_.param<std::string>("command_topic", commandTopic, "/command");
    commandPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(commandTopic, 1);

    std::string pathTopic;
    nh_.param<std::string>("path_topic", pathTopic, "/desired_path");
    targetPathSubscriber_ = nh_.subscribe(pathTopic, 10, &MpcController::pathCallback, this);

    observationPublisher_ =
        nh_.advertise<ocs2_msgs::mpc_observation>("/" + robotName + "_mpc_observation", 10);

    command_path_publisher_.init(nh_, "/command_path", 10);
    rollout_publisher_.init(nh_, "/mpc_rollout", 10);

    observation_.time = ros::Time::now().toSec();
    observation_.state.setZero(ocs2::mobile_manipulator::STATE_DIM(ArmInputDim));
    observation_.input.setZero(ocs2::mobile_manipulator::INPUT_DIM(ArmInputDim));
    positionCommand_.setZero();
    velocityCommand_.setZero();
    unloaded_ = false;
    stopped_ = true;

    // get static transform from tool to tracked MPC frame
    geometry_msgs::TransformStamped transform;
    try {
      // target_frame, source_frame ...
      transform = tf_buffer_.lookupTransform(tool_link_, mm_interface_->getEEFrame(), ros::Time(0),
                                             ros::Duration(3.0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return false;
    }
    tf::transformMsgToEigen(transform.transform, T_tool_ee_);
    ROS_INFO_STREAM("Transform from " << mm_interface_->getEEFrame() << " to " << tool_link_ << " is"
                                      << std::endl
                                      << T_tool_ee_.matrix());

    ROS_INFO("[MpcController::init] Initializing threads.");
    mpcThread_ = std::thread(&MpcController::advanceMpc, this);

    ROS_INFO("[MpcController::init] MPC Controller successfully initialized.");
    return true;
  }

  void start(const state_vector_t& initial_observation) {
    // initial observation
    if (!stopped_) return;

    // flags
    policyReady_ = false;
    referenceEverReceived_ = false;
    observationEverReceived_ = false;

    // mpc problem
    mm_interface_.reset(
        new ocs2::mobile_manipulator::MobileManipulatorInterface(taskFile_, urdfXML_, ArmInputDim, baseType_));
    mpcPtr_ = mm_interface_->getMpc();

    // mpc solution update thread
    mpc_mrt_interface_.reset(new ocs2::MPC_MRT_Interface(*mpcPtr_));
    mpc_mrt_interface_->reset();

    mpcTimer_.reset();

    ROS_INFO("[MpcController::start] Starting controller.");
    stopped_ = false;

    startTime_ = ros::Time::now().toSec();
    initialState_ = initial_observation;
    setObservation(initial_observation);

    ROS_INFO("[MpcController::start] Successfully started.");
  }

  void update(const ros::Time& time, const state_vector_t& observation) {
    setObservation(observation);
    updateCommand();
    publishRos();
  }

  void stop() {
    ROS_INFO("[MPC_Controller::stop] Stopping MPC update thread");
    stopped_ = true;
    ROS_INFO("[MPC_Controller::stop] Stopped MPC update thread");
  }

  /**
   * Updates the desired cost trajectories from path message.
   * @param desiredPath
   */
  void pathCallback(const nav_msgs::PathConstPtr& desiredPath) {
    if (desiredPath->poses.empty()) {
      //ROS_WARN("[MPC_Controller::pathCallback] Received path is empty");
      return;
    }

    bool ok = sanityCheck(*desiredPath);
    if (!ok) {
      //ROS_WARN("[MPC_Controller::pathCallback] Received path is ill formed.");
      return;
    }

    //ROS_DEBUG_STREAM("[MPC_Controller::pathCallback] Received new path with "
    //                << desiredPath->poses.size() << " poses.");

    {
      std::lock_guard<std::mutex> lock(desiredPathMutex_);
      desiredPath_ = *desiredPath;
      transformPath(desiredPath_);   // transform to the correct base frame
      adjustPathTime(desiredPath_);  // adjust time stamps keeping relative time-distance
    }
    referenceEverReceived_ = true;
  }

  inline state_vector_t getPositionCommand() const { return positionCommand_; }
  inline input_vector_t getVelocityCommand() const { return velocityCommand_; }
  /*inline joint_vector_t getArmPositionCommand() const { return positionCommand_.tail<ocs2::mobile_manipulator::ARM_INPUT_DIM>(); }
  inline joint_vector_t getArmVelocityCommand() const { return velocityCommand_.tail<ocs2::mobile_manipulator::ARM_INPUT_DIM>(); }
  inline base_vector_t getBasePositionCommand() const { return positionCommand_.head<ocs2::mobile_manipulator::BASE_INPUT_DIM>(); }
  inline base_vector_t getBaseVelocityCommand() const { return velocityCommand_.head<ocs2::mobile_manipulator::BASE_INPUT_DIM>(); }*/
  inline const ros::NodeHandle& getNodeHandle() { return nh_; }

 protected:

  /**
   * Optional preprocessing step for the tracked path.
   * @param desiredPath
   */
  virtual void adjustPath(nav_msgs::Path& desiredPath) {};

 private:
  void advanceMpc() {
    while (!unloaded_) {
      while (stopped_) {
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1e3 / mpcFrequency_)));
      }

      if (!referenceEverReceived_) {
        //ROS_WARN_THROTTLE(3.0, "Reference never received. Skipping MPC update.");
        continue;
      }

      if (!observationEverReceived_) {
        //ROS_WARN_THROTTLE(3.0, "Observation never received. Skipping MPC update.");
        continue;
      }

      nav_msgs::Path pathTarget;
      {
        std::lock_guard<std::mutex> lock(desiredPathMutex_);
        pathTarget = desiredPath_;
      }

      adjustPath(pathTarget);
      writeDesiredPath(pathTarget);

      if (command_path_publisher_.trylock()) {
        command_path_publisher_.msg_ = pathTarget;
        command_path_publisher_.unlockAndPublish();
      }

      {
        std::lock_guard<std::mutex> lock(observationMutex_);
        mpc_mrt_interface_->setCurrentObservation(observation_);
      }

      mpcTimer_.startTimer();
      try {
        mpc_mrt_interface_->advanceMpc();
      } catch (const std::runtime_error& exc) {
        ROS_ERROR_STREAM(exc.what());
      }
      mpcTimer_.endTimer();
      //ROS_INFO_STREAM_THROTTLE(2.0, "Mpc update took " << mpcTimer_.getLastIntervalInMilliseconds() << " ms.");
      policyReady_ = true;
      //std::this_thread::sleep_for(std::chrono::milliseconds((int)(1e3 / mpcFrequency_)));
    }
  }

  void setObservation(const state_vector_t& q) {
    std::unique_lock<std::mutex> lock(observationMutex_);
    observation_.time = ros::Time::now().toSec() - startTime_;
    observation_.state = q;
    observationEverReceived_ = true;
  }

  void updateCommand() {
    static Eigen::VectorXd mpcState;
    static Eigen::VectorXd mpcInput;
    static size_t mode;

    if (!referenceEverReceived_ || !policyReady_) {
      positionCommand_ = initialState_;
      velocityCommand_.setZero();
      return;
    }

    {
      std::unique_lock<std::mutex> lock(policyMutex_);
      mpc_mrt_interface_->updatePolicy();
      mpc_mrt_interface_->evaluatePolicy(observation_.time, observation_.state, mpcState, mpcInput,
                                         mode);
    }
    positionCommand_ = mpcState;
    velocityCommand_ = mpcInput;
  }

  void writeDesiredPath(const nav_msgs::Path& desiredPath) {
    ocs2::TargetTrajectories targetTrajectories(desiredPath.poses.size());
    size_t idx = 0;
    for (const auto& waypoint : desiredPath.poses) {
      // Desired state trajectory
      ocs2::scalar_array_t& tDesiredTrajectory = targetTrajectories.timeTrajectory;
      tDesiredTrajectory[idx] = waypoint.header.stamp.toSec() - startTime_;

      // Desired state trajectory
      ocs2::vector_array_t& xDesiredTrajectory = targetTrajectories.stateTrajectory;
      xDesiredTrajectory[idx].resize(7);
      xDesiredTrajectory[idx].head<3>() << waypoint.pose.position.x,
          waypoint.pose.position.y, waypoint.pose.position.z;
      xDesiredTrajectory[idx][3] = waypoint.pose.orientation.x;
      xDesiredTrajectory[idx][4] = waypoint.pose.orientation.y;
      xDesiredTrajectory[idx][5] = waypoint.pose.orientation.z;
      xDesiredTrajectory[idx][6] = waypoint.pose.orientation.w;


      // Desired input trajectory
      ocs2::vector_array_t& uDesiredTrajectory = targetTrajectories.inputTrajectory;
      uDesiredTrajectory[idx].setZero(9);
      idx++;
    }

    mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
  }

  void publishObservation() {
    {
      std::lock_guard<std::mutex> lock(observationMutex_);
      observation_tmp_ = observation_;
    }
    ocs2_msgs::mpc_observation observationMsg =
        ocs2::ros_msg_conversions::createObservationMsg(observation_tmp_);
    observationPublisher_.publish(observationMsg);
  }

  // path processing
  static bool sanityCheck(const nav_msgs::Path& path) {
    // check that path adheres to conventional frame
    if (path.header.frame_id != "world") {
      ROS_ERROR("[MpcController::transformPath] Desired path must be in world frame.");
      return false;
    }

    // check time monotonicity
    for (size_t idx = 1; idx < path.poses.size(); idx++) {
      if ((path.poses[idx].header.stamp.toSec() - path.poses[idx - 1].header.stamp.toSec()) <= 0) {
        return false;
      }
    }
    return true;
  }

  void transformPath(nav_msgs::Path& desiredPath) {
    for (auto& pose : desiredPath.poses) {
      tf::poseMsgToEigen(pose.pose, T_x_tool_);
      T_base_ee_ = T_x_tool_ * T_tool_ee_;
      tf::poseEigenToMsg(T_base_ee_, pose.pose);
    }
  }

  void adjustPathTime(nav_msgs::Path& desiredPath) const {
    if (desiredPath.poses.empty()) return;

    // take only relative timing from path
    //  double current_mpc_time = ros::Time::now().toSec() - start_time_;
    //  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Current mpc time: " << current_mpc_time);
    //
    //  double time_offset = desiredPath.poses[0].header.stamp.toSec() - current_mpc_time;
    //  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Time offset is: " << time_offset);
    //
    //  for (auto& pose : desiredPath.poses) {
    //    pose.header.stamp = pose.header.stamp - ros::Duration(time_offset);
    //  }
  }

  // publish ros
  void publishRos() {
    //publishDesiredPath();
    publishCurrentRollout();
    publishObservation();
  }

  void publishCurrentRollout() {
    ocs2::scalar_array_t time_trajectory;
    ocs2::vector_array_t state_trajectory;
    {
      std::unique_lock<std::mutex> lock(policyMutex_);
      if (!mpc_mrt_interface_->initialPolicyReceived()) return;

      try {
        time_trajectory = mpc_mrt_interface_->getPolicy().timeTrajectory_;
        state_trajectory = mpc_mrt_interface_->getPolicy().stateTrajectory_;
      }
      catch(std::runtime_error& err){
        //ROS_WARN_STREAM_THROTTLE(1.0, err.what());
        return;
      }
    }

    std::string ee_frame = mm_interface_->getEEFrame();  // this is the one in the task file
    nav_msgs::Path rollout;
    rollout.header.frame_id = "world";
    rollout.header.stamp = ros::Time::now();
    for (int i = 0; i < time_trajectory.size(); i++) {
      auto& data = mm_interface_->getPinocchioDesiredInterface().getData();
      const auto& model = mm_interface_->getPinocchioDesiredInterface().getModel();

      pinocchio::forwardKinematics(model, data, state_trajectory[i]);
      pinocchio::updateFramePlacements(model, data);

      int frameId = model.getBodyId(mm_interface_->getEEFrame());
      Eigen::Vector3d t = data.oMf[frameId].translation();
      Eigen::Quaterniond q = Eigen::Quaterniond(data.oMf[frameId].rotation());

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = t.x();
      pose.pose.position.y = t.y();
      pose.pose.position.z = t.z();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      rollout.poses.push_back(pose);
    }

    if (rollout_publisher_.trylock()){
      rollout_publisher_.msg_ = rollout;
      rollout_publisher_.unlockAndPublish();
    }
  }

  void publishDesiredPath() {
    // TODO(giuseppe) to implement
  }

 protected:
  double startTime_;

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
  std::atomic_bool unloaded_;
  std::atomic_bool stopped_;
  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;
  double mpcFrequency_;

  std::atomic_bool policyReady_;
  std::atomic_bool referenceEverReceived_;
  std::atomic_bool observationEverReceived_;

  std::string taskFile_;
  std::string urdfXML_;
  ocs2::mobile_manipulator::BaseType baseType_;

  std::thread mpcThread_;
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  std::mutex observationMutex_;
  ocs2::SystemObservation observation_;
  ocs2::SystemObservation observation_tmp_;

  std::mutex desiredPathMutex_;
  nav_msgs::Path desiredPath_;

  std::mutex policyMutex_;
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> mm_interface_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Eigen::Affine3d T_tool_ee_;   // transform from frame tracked by MPC to the actual tool frame
  Eigen::Affine3d T_x_tool_;    // transfrom from point in the path to the path reference frame
  Eigen::Affine3d T_base_ee_;   // transform from base to desired end effector pose

  // realtime publisher
  double publishRosFrequency_;
  std::thread publishRosThread_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> command_path_publisher_;
  realtime_tools::RealtimePublisher<nav_msgs::Path> rollout_publisher_;

};  
}  // namespace mobile_manipulator