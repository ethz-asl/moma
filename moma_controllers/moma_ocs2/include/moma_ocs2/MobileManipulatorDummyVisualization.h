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

#pragma once
//clang-format off
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
//clang-format on

#include <moma_ocs2/MobileManipulatorInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorDummyVisualization final : public DummyObserver {
 public:
  MobileManipulatorDummyVisualization(ros::NodeHandle& nodeHandle,
                                      const MobileManipulatorInterface& interface)
      : pinocchioInterface_(interface.getPinocchioInterface()) {
    launchVisualizerNode(nodeHandle);
  }

  ~MobileManipulatorDummyVisualization() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& policy,
              const CommandData& command) override;

 private:
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);
  void publishTargetTrajectories(const ros::Time& timeStamp,
                                 const TargetTrajectories& targetTrajectories);
  void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

  PinocchioInterface pinocchioInterface_;

  tf::TransformBroadcaster tfBroadcaster_;

  sensor_msgs::JointState jointState_;
  ros::Publisher jointStatePublisher_;
  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher stateOptimizedPosePublisher_;

  std::string endEffectorName_;
  size_t armJoints_;
  std::vector<std::string> jointNames_;
  std::map<std::string, scalar_t> jointPositions_;
  std::unique_ptr<GeometryInterfaceVisualization> geometryVisualization_;
};

// TODO(mspieler): move somewhere else
Eigen::VectorXd getJointPositions(Eigen::VectorXd state);
Eigen::Vector3d getBasePosition(Eigen::VectorXd state);
Eigen::Quaterniond getBaseOrientation(Eigen::VectorXd state);

}  // namespace mobile_manipulator
}  // namespace ocs2
