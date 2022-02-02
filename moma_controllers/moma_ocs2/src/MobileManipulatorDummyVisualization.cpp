/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/package.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <moma_ocs2/MobileManipulatorDummyVisualization.h>
#include <moma_ocs2/MobileManipulatorInterface.h>
#include <moma_ocs2/definitions.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd getJointPositions(Eigen::VectorXd state) {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Vector3d getBasePosition(Eigen::VectorXd state) {
  Eigen::Vector3d position;
  position << state(0), state(1), 0.0;
  return position;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Quaterniond getBaseOrientation(Eigen::VectorXd state) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  const std::string urdfName = "/ocs2_mpc/robot_description_ocs2";
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    ROS_ERROR("URDF model load was NOT successful");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }

  urdf::LinkConstSharedPtr parentLink = model.getRoot();
  urdf::LinkConstSharedPtr childLink = nullptr;
  while (true){
    if (parentLink->child_joints.empty()){
      std::cout << "End of tree reached." << std::endl;
      break;
    }

    bool movableJointFound = false;
    for (const auto& joint : parentLink->child_joints) {
      std::cout << "Joint: " << joint->name << std::endl;
      if (joint->type != urdf::Joint::FIXED) {
        std::cout << "Adding joint: " << joint->name << std::endl;
        jointPositions_[joint->name] = 0.0;
        jointNames_.push_back(joint->name);
        jointState_.name.push_back(joint->name);
        jointState_.position.push_back(0.0);
        childLink = model.links_[joint->child_link_name];
        movableJointFound = true;
        break;
      }
    }

    if (movableJointFound)
      parentLink = childLink;
    else
      parentLink = parentLink->child_links[0];

  }
  armJoints_ = jointPositions_.size();

  jointStatePublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);

  nodeHandle.param<std::string>("end_effector_name", endEffectorName_, "WRIST_2");

  std::string robotDescription;
  nodeHandle.param<std::string>(urdfName, robotDescription, "");

  PinocchioInterface pinocchioInterface = MobileManipulatorInterface::buildPinocchioInterfaceFromXML(robotDescription);
  // TODO(giuseppe) get the collision pairs from the task.info file to match the current mpc setup
  PinocchioGeometryInterface geomInterface(pinocchioInterface, {{1, 4}, {1, 6}});
  geometryVisualization_.reset(new GeometryInterfaceVisualization(std::move(pinocchioInterface), geomInterface, nodeHandle));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy,
                                                 const CommandData& command) {
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
  publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(timeStamp, policy);
  geometryVisualization_->publishDistances(observation.state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation) {
  // publish all joints transforms
  int idx = 0;
  const auto j_arm = getJointPositions(observation.state);
  for (const auto& jointName : jointNames_){
    jointPositions_[jointName] = j_arm[idx];
    jointState_.name[idx] = jointName;
    jointState_.position[idx] = j_arm[idx];
    idx++;
  }
  jointState_.header.stamp = ros::Time::now();
  jointStatePublisher_.publish(jointState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishTargetTrajectories(const ros::Time& timeStamp,
                                                                    const TargetTrajectories& targetTrajectories) {
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition = targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = targetTrajectories.stateTrajectory.back().tail(4);
  geometry_msgs::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "odom";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
  tfBroadcaster_.sendTransform(command_tf);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto& mpcStateTrajectory = policy.stateTrajectory_;

  visualization_msgs::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  std::vector<geometry_msgs::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    const auto eeIndex = model.getBodyId(endEffectorName_);
    const vector_t eePosition = data.oMf[eeIndex].translation();
    endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(getBasePosition(state));
    pose.orientation = ros_msg_helpers::getOrientationMsg(getBaseOrientation(state));
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("odom", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("odom", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace mobile_manipulator
}  // namespace ocs2
