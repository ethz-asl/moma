#include <moma_ocs2/definitions.h>
#include <moma_ocs2/constraint/DefaultConfigurationConstraint.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <iostream>

namespace ocs2 {
namespace mobile_manipulator {

size_t DefaultConfigurationConstraint::getNumConstraints(scalar_t time) const {
  return armInputDim_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t DefaultConfigurationConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  vector_t constraint(armInputDim_);
  for (size_t i{}; i<armInputDim_; i++) {
      constraint[i] = state(BASE_INPUT_DIM + i) - defaultConfiguration_[i];
  }
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation DefaultConfigurationConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                         const PreComputation& preComputation) const {
  
  VectorFunctionLinearApproximation limits(armInputDim_, STATE_DIM(armInputDim_), INPUT_DIM(armInputDim_));
  limits.f = state.tail(armInputDim_);
  limits.dfdx.setZero();
  for (size_t i{}; i<armInputDim_; i++) limits.dfdx(i, BASE_INPUT_DIM + i) = scalar_t(1.0);
  limits.dfdu.setZero();
  return limits;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
