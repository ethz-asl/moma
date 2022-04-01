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

#include <moma_ocs2/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  // Initialize ros node
  ros::init(argc, argv, "mobile_manipulator_mpc");
  ros::NodeHandle nodeHandle;

  const int armInputDim = 7;

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
  const std::string libFolder = ros::package::getPath("moma_ocs2") + "/auto_generated";

  // Robot interface
  MobileManipulatorInterface interface(taskFile, urdfXML, armInputDim, baseType);

  // ROS ReferenceManager
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(
      new ocs2::RosReferenceManager("mobile_manipulator", interface.getReferenceManagerPtr()));
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  ocs2::MPC_DDP mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                    interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, "mobile_manipulator");
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
