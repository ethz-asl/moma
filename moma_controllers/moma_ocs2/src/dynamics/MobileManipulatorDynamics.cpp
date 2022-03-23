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

#include <moma_ocs2/MobileManipulatorDynamics.h>

namespace ocs2 {
namespace mobile_manipulator {

MobileManipulatorDynamics::MobileManipulatorDynamics(const std::string& modelName, const size_t armInputDim, const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                     bool recompileLibraries /*= true*/, const BaseType baseType /*= BaseType::none */,
                                                     bool verbose /*= true*/)
    : SystemDynamicsBaseAD() {
  armInputDim_ = armInputDim;
  baseType_ = baseType;
  Base::initialize(STATE_DIM(armInputDim), INPUT_DIM(armInputDim), modelName, modelFolder, recompileLibraries, verbose);
}

ad_vector_t MobileManipulatorDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                     const ad_vector_t& parameters) const {
  ad_vector_t dxdt(STATE_DIM(armInputDim_));
  const auto theta = state(2);
  typename ad_vector_t::Scalar vx;      // forward velocity in base frame
  typename ad_vector_t::Scalar vy;      // sideways velocity in base frame
  typename ad_vector_t::Scalar vtheta;  // angular velocity
  switch(baseType_) {
    case BaseType::holonomic:
      vx = input(0);
      vy = input(1);
      vtheta = input(2);
      break;
    case BaseType::skidsteer:
      vx = input(0);
      vy = typename ad_vector_t::Scalar(0.0);
      vtheta = input(2);
      break;
    case BaseType::none:
    default:
      vx = typename ad_vector_t::Scalar(0.0);
      vy = typename ad_vector_t::Scalar(0.0);
      vtheta = typename ad_vector_t::Scalar(0.0);
      break;
  }
  dxdt << cos(theta) * vx - sin(theta) * vy, sin(theta) * vx + cos(theta) * vy, vtheta, input.tail(armInputDim_);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
