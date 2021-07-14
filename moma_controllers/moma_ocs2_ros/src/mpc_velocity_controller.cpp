//
// Created by giuseppe on 31.12.20.
//

#include "moma_ocs2_ros/mpc_velocity_controller.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>

namespace moma_controllers {

MpcController::MpcController(const ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {}

bool MpcController::init() {
  std::string robotName;
  nh_.param<std::string>("/mpc_controller/robot_name", robotName, "mobile_manipulator");
  taskFile_  = ros::package::getPath("moma_ocs2") + "/config/mpc/task_" + robotName + ".info";

  mm_interface_.reset(
      new ocs2::mobile_manipulator::MobileManipulatorInterface(taskFile_, robotName));
  mpcPtr_ = mm_interface_->getMpc();
  //mpc_mrt_interface_.reset(new ocs2::MPC_MRT_Interface(*mpcPtr_));

  nh_.param<std::string>("/robot_description_mpc", robot_description_, "");
  nh_.param<std::string>("/mpc_controller/base_link", base_link_, "base_link");
  nh_.param<std::string>("/mpc_controller/tool_link", tool_link_, "tool_frame");
  nh_.param<double>("/mpc_controller/mpc_frequency", mpcFrequency_, -1);
  nh_.param<double>("/mpc_controller/publish_ros_frequency", publishRosFrequency_, 20);

  std::string commandTopic;
  nh_.param<std::string>("/mpc_controller/command_topic", commandTopic, "/command");
  commandPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(commandTopic, 1);

  std::string pathTopic;
  nh_.param<std::string>("/mpc_controller/path_topic", pathTopic, "/desired_path");
  targetPathSubscriber_ = nh_.subscribe(pathTopic, 10, &MpcController::pathCallback, this);

  observationPublisher_ =
      nh_.advertise<ocs2_msgs::mpc_observation>("/" + robotName + "_mpc_observation", 10);

  command_path_publisher_.init(nh_, "/command_path", 10);
  rollout_publisher_.init(nh_, "/mpc_rollout", 10);

  observation_.time = 0.0;
  observation_.state.setZero(10);
  observation_.input.setZero(9);
  positionCommand_.setZero(10);
  velocityCommand_.setZero(9);
  stopped_ = true;

  // get static transform from tool to tracked MPC frame
  geometry_msgs::TransformStamped transform;
  try {
    // target_frame, source_frame ...
    transform = tf_buffer_.lookupTransform(tool_link_, mm_interface_->eeFrame_, ros::Time(0),
                                           ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  tf::transformMsgToEigen(transform.transform, T_tool_ee_);
  ROS_INFO_STREAM("Transform from " << mm_interface_->eeFrame_ << " to " << tool_link_ << " is"
                                    << std::endl
                                    << T_tool_ee_.matrix());

  ROS_INFO("MPC Controller successfully initialized.");
  return true;
}

MpcController::~MpcController(){
  publishRosThread_.join();
  mpcThread_.join();
}

void MpcController::start(const joint_vector_t& initial_observation) {
  // initial observation
  if (!stopped_) return;

  // flags
  policyReady_ = false;
  referenceEverReceived_ = false;
  observationEverReceived_ = false;

  jointInitialState_ = initial_observation;
  setObservation(initial_observation);

  // mpc solution update thread
  start_time_ = 0.0;
  mpc_mrt_interface_.reset(new ocs2::MPC_MRT_Interface(*mpcPtr_));
  mpc_mrt_interface_->reset();
  
  mpcTimer_.reset();

  stopped_ = false;
  mpcThread_ = std::thread(&MpcController::advanceMpc, this);
  publishRosThread_ = std::thread(&MpcController::publishRos, this);

  ROS_INFO("MPC Controller Successfully started.");
}

void MpcController::advanceMpc() {
  while (ros::ok() && !stopped_) {
    if (!referenceEverReceived_) {
      ROS_WARN_THROTTLE(3.0, "Reference never received. Skipping MPC update.");
      continue;
    }

    if (!observationEverReceived_) {
      ROS_WARN_THROTTLE(3.0, "Observation never received. Skipping MPC update.");
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
    ROS_INFO_STREAM_THROTTLE(10.0, "Mpc update took " << mpcTimer_.getLastIntervalInMilliseconds() << " ms.");
    policyReady_ = true;
  }
}

void MpcController::update(const ros::Time& time, const joint_vector_t& observation) {
  std::unique_lock<std::mutex> lock(observationMutex_);
  setObservation(observation);
  updateCommand();
}

void MpcController::setObservation(const joint_vector_t& observation) {
  observation_.time = ros::Time::now().toSec() - start_time_;
  observation_.state.tail(7) = observation;
  observationEverReceived_ = true;
}

void MpcController::publishObservation() {
  ocs2_msgs::mpc_observation observationMsg;
  ocs2::ros_msg_conversions::createObservationMsg(observation_, observationMsg);
  observationPublisher_.publish(observationMsg);
}

void MpcController::updateCommand() {
  static Eigen::VectorXd mpcState;
  static Eigen::VectorXd mpcInput;
  static size_t mode;

  if (!referenceEverReceived_ || !policyReady_) {
    positionCommand_ = jointInitialState_;
    velocityCommand_.setZero();
    return;
  }

  mpc_mrt_interface_->updatePolicy();
  mpc_mrt_interface_->evaluatePolicy(observation_.time, observation_.state, mpcState, mpcInput,
                                       mode);
  positionCommand_ = mpcState.tail(7);
  velocityCommand_ = mpcInput.tail(7);
}

void MpcController::adjustPathTime(nav_msgs::Path& desiredPath) const {
  if (desiredPath.poses.empty()) return;

  // take only relative timing from path
  double current_mpc_time = ros::Time::now().toSec() - start_time_;
  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Current mpc time: " << current_mpc_time);

  double time_offset = desiredPath.poses[0].header.stamp.toSec() - current_mpc_time;
  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Time offset is: " << time_offset);

  for (auto& pose : desiredPath.poses) {
    pose.header.stamp = pose.header.stamp - ros::Duration(time_offset);
  }
}

void MpcController::pathCallback(const nav_msgs::PathConstPtr& desiredPath) {
  if (desiredPath->poses.empty()) {
    ROS_WARN("[MPC_Controller::pathCallback] Received path is empty");
    return;
  }

  bool ok = sanityCheck(*desiredPath);
  if (!ok) {
    ROS_WARN("[MPC_Controller::pathCallback] Received path is ill formed.");
    return;
  }

  ROS_INFO_STREAM("[MPC_Controller::pathCallback] Received new path with "
                  << desiredPath->poses.size() << " poses.");

  {
    std::lock_guard<std::mutex> lock(desiredPathMutex_);
    desiredPath_ = *desiredPath;
    transformPath(desiredPath_);   // transform to the correct base frame
    adjustPathTime(desiredPath_);  // adjust time stamps keeping relative time-distance
  }
  referenceEverReceived_ = true;
}

void MpcController::writeDesiredPath(const nav_msgs::Path& desiredPath) {
  ocs2::CostDesiredTrajectories costDesiredTrajectories(desiredPath.poses.size());
  size_t idx = 0;
  for (const auto& waypoint : desiredPath.poses) {
    // Desired state trajectory
    ocs2::scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
    tDesiredTrajectory[idx] = waypoint.header.stamp.toSec();

    // Desired state trajectory
    ocs2::vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
    xDesiredTrajectory[idx].resize(7);
    xDesiredTrajectory[idx].template tail<4>() =
        Eigen::Quaterniond(waypoint.pose.orientation.w, waypoint.pose.orientation.x,
                           waypoint.pose.orientation.y, waypoint.pose.orientation.z)
            .coeffs();
    xDesiredTrajectory[idx].template head<3>() << waypoint.pose.position.x,
        waypoint.pose.position.y, waypoint.pose.position.z;

    // Desired input trajectory
    ocs2::vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
    uDesiredTrajectory[idx].setZero(9);
    idx++;
  }

  mpc_mrt_interface_->setTargetTrajectories(costDesiredTrajectories);
}

bool MpcController::sanityCheck(const nav_msgs::Path& path) {
  // check time monotonicity
  for (size_t idx = 1; idx < path.poses.size(); idx++) {
    if ((path.poses[idx].header.stamp.toSec() - path.poses[idx - 1].header.stamp.toSec()) < 0) {
      return false;
    }
  }
  return true;
}

void MpcController::transformPath(nav_msgs::Path& desiredPath) {
  ROS_INFO_STREAM("[MPC_Controller::transformPath] Transforming path from "
                  << desiredPath.header.frame_id << " to " << base_link_);
  geometry_msgs::TransformStamped transformStamped;
  try {
    // target_frame, source_frame ...
    transformStamped = tf_buffer_.lookupTransform(base_link_, desiredPath.header.frame_id,
                                                  ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
  tf::transformMsgToEigen(transformStamped.transform, T_base_x_);

  ros::Time stamp;
  desiredPath.header.frame_id = base_link_;
  for (auto& pose : desiredPath.poses) {
    tf::poseMsgToEigen(pose.pose, T_x_tool_);
    T_base_ee_ = T_base_x_ * T_x_tool_ * T_tool_ee_;
    tf::poseEigenToMsg(T_base_ee_, pose.pose);
  }
}

void MpcController::stop() {
  ROS_INFO("[MPC_Controller::stop] Stopping MPC update thread");
  stopped_ = true;
  mpcThread_.join();
  publishRosThread_.join();
  ROS_INFO("[MPC_Controller::stop] Stopped MPC update thread");
}

void MpcController::publishCurrentRollout(){
  // TODO(giuseppe) replace with something that returns simply the state trajectory
 //  ocs2::scalar_array_t time_trajectory;
 //  ocs2::vector_array_t state_trajectory;
 //  {
 //    std::unique_lock<std::mutex> lock(policyMutex_);
 //    if (!mpc_mrt_interface_->initialPolicyReceived()) return;

 //    try{
	//   time_trajectory = mpc_mrt_interface_->getPolicy().timeTrajectory_;
	//   state_trajectory = mpc_mrt_interface_->getPolicy().stateTrajectory_;
	// }
 //    catch(std::runtime_error& err){
 //      ROS_WARN_STREAM_THROTTLE(1.0, err.what());
 //      return;	
 //    }
 //  }

 //  // TODO(giuseppe) hard coded -> remove
 //  std::string ee_frame = "end_effector_link";  // this is the one in the task file
 //  nav_msgs::Path rollout;
 //  rollout.header.frame_id = "arm_base_link";
 //  rollout.header.stamp = ros::Time::now();
 //  for (int i = 0; i < time_trajectory.size(); i++) {
 //    model_->updateState(state_trajectory[i].tail<7>(), Eigen::VectorXd::Zero(model_->getDof()));
 //    Eigen::Vector3d t(model_->getFramePlacement(ee_frame).translation());
 //    Eigen::Quaterniond q(model_->getFramePlacement(ee_frame).rotation());

 //    geometry_msgs::PoseStamped pose;
 //    pose.header.frame_id = "arm_base_link";
 //    pose.pose.position.x = t.x();
 //    pose.pose.position.y = t.y();
 //    pose.pose.position.z = t.z();
 //    pose.pose.orientation.x = q.x();
 //    pose.pose.orientation.y = q.y();
 //    pose.pose.orientation.z = q.z();
 //    pose.pose.orientation.w = q.w();
 //    rollout.poses.push_back(pose);
 //  }

 //  if (rollout_publisher_.trylock()){
 //    rollout_publisher_.msg_ = rollout;
 //    rollout_publisher_.unlockAndPublish();
 //  }
}

void MpcController::publishDesiredPath(){
  // TODO(giuseppe) to implement
}

void MpcController::publishRos(){
  while (ros::ok() && !stopped_){
    //publishDesiredPath();
    publishCurrentRollout();
    publishObservation();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(1e3 / publishRosFrequency_)));
  }
}
}  // namespace kinova_controllers
