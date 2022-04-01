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

#include <moma_ocs2/MobileManipulatorDummyVisualization.h>
#include <moma_ocs2/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  const int armInputDim = 7;

  // Initialize ros node
  ros::init(argc, argv, "mobile_manipulator_mrt");
  ros::NodeHandle nodeHandle;

  // Params
  std::string taskFile;
  if (!nodeHandle.param("/ocs2_mpc/task_file", taskFile, {})) {
    ROS_ERROR("Failed to retrieve /ocs2_mpc/task_file from param server.");
    return 0;
  }
  std::string urdfXML;
  if (!nodeHandle.param("/ocs2_mpc/robot_description_ocs2", urdfXML, {})) {
    ROS_ERROR("Failed to retrieve /ocs2_mpc/robot_description_ocs2 from param server.");
    return 0;
  }

  int baseTypeInt;
  if (!nodeHandle.param("/ocs2_mpc/base_type", baseTypeInt, 0)) {
    ROS_ERROR("Failed to retrieve /ocs2_mpc/base_type from param server.");
    return 0;
  }
  if (baseTypeInt >= BASE_TYPE_COUNT) {
    ROS_ERROR("The value of base_type is not supported.");
    return 0;
  }
  BaseType baseType = static_cast<BaseType>(baseTypeInt);

  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFile, urdfXML, armInputDim,
                                                           baseType);

  // MRT
  MRT_ROS_Interface mrt("mobile_manipulator");
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<mobile_manipulator::MobileManipulatorDummyVisualization> dummyVisualization(
      new mobile_manipulator::MobileManipulatorDummyVisualization(nodeHandle, interface));

  // Dummy MRT
  MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                           interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(mobile_manipulator::INPUT_DIM(armInputDim));
  initObservation.time = 0.0;

  // initial command
  vector_t initTarget(7);
  initTarget.head(3) << 0.5, 0.0, 0.8;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(0.0, 0.7071068, 0, 0.7071068).coeffs();
  const vector_t zeroInput = vector_t::Zero(mobile_manipulator::INPUT_DIM(armInputDim));
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget},
                                                  {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
