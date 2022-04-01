#pragma once

#include <moma_ocs2/definitions.h>
#include <ocs2_core/constraint/StateConstraint.h>

#include <memory>

namespace ocs2 {
namespace mobile_manipulator {

class JointPositionLimits final : public StateConstraint {
 public:
  JointPositionLimits(const size_t armInputDim)
      : StateConstraint(ConstraintOrder::Linear), armInputDim_(armInputDim) {}
  ~JointPositionLimits() override = default;
  JointPositionLimits* clone() const override { return new JointPositionLimits(*this); }

  size_t getNumConstraints(scalar_t time) const override { return INPUT_DIM(armInputDim_); }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation&) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation&) const override;

 private:
  size_t armInputDim_;
  JointPositionLimits(const JointPositionLimits& other) = default;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
