/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <moma_ocs2/definitions.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {
namespace mobile_manipulator {

template <typename SCALAR>
class MobileManipulatorPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
 private:
  BaseType baseType_;
  size_t armInputDim_;

 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  MobileManipulatorPinocchioMapping(const size_t armInputDim,
                                    const BaseType baseType = BaseType::none)
      : armInputDim_(armInputDim), baseType_(baseType){};
  ~MobileManipulatorPinocchioMapping() override = default;
  MobileManipulatorPinocchioMapping<SCALAR>* clone() const override {
    return new MobileManipulatorPinocchioMapping<SCALAR>(*this);
  }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override {
    vector_t dxdt(STATE_DIM(armInputDim_));
    const auto theta = state(2);
    typename vector_t::Scalar vx;      // forward velocity in base frame
    typename vector_t::Scalar vy;      // sideways velocity in base frame
    typename vector_t::Scalar vtheta;  // angular velocity
    switch (baseType_) {
      case BaseType::holonomic:
        vx = input(0);
        vy = input(1);
        vtheta = input(2);
        break;
      case BaseType::skidsteer:
        vx = input(0);
        vy = typename vector_t::Scalar(0.0);
        vtheta = input(2);
        break;
      case BaseType::none:
      default:
        vx = typename vector_t::Scalar(0.0);
        vy = typename vector_t::Scalar(0.0);
        vtheta = typename vector_t::Scalar(0.0);
    }
    dxdt << cos(theta) * vx - sin(theta) * vy, sin(theta) * vx + cos(theta) * vy, vtheta,
        input.tail(armInputDim_);
    return dxdt;
  }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq,
                                                const matrix_t& Jv) const override {
    matrix_t dfdu(Jv.rows(), INPUT_DIM(armInputDim_));
    Eigen::Matrix<SCALAR, 3, 3> dvdu_base;
    const SCALAR theta = state(2);
    switch (baseType_) {
      case BaseType::holonomic:
        // clang-format off
        dvdu_base << cos(theta), -sin(theta), 0.0,
                     sin(theta), cos(theta), 0.0,
                     0.0, 0.0, 1.0;
        break;
      case BaseType::skidsteer:
        // clang-format off
        dvdu_base << cos(theta), 0.0, 0.0,
                     sin(theta), 0.0, 0.0,
                     0.0, 0.0, 1.0;
        break;
      case BaseType::none:
      default:
        // clang-format off
        dvdu_base.setZero();
    }

    // clang-format on
    dfdu.template leftCols<3>() = Jv.template leftCols<3>() * dvdu_base;
    dfdu.template rightCols(armInputDim_) = Jv.template rightCols(armInputDim_);
    return {Jq, dfdu};
  }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
