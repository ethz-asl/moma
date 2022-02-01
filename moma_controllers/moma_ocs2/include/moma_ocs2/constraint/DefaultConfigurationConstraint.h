#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2 {
namespace mobile_manipulator {

class DefaultConfigurationConstraint final : public StateConstraint {
  using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
 public:
  DefaultConfigurationConstraint(const size_t armInputDim, 
                                 const vector_t& defaultConfiguration): 
                                 StateConstraint(ConstraintOrder::Linear),
                                 armInputDim_(armInputDim), defaultConfiguration_(defaultConfiguration){};
  ~DefaultConfigurationConstraint() override = default;
  DefaultConfigurationConstraint* clone() const override { return new DefaultConfigurationConstraint(armInputDim_, defaultConfiguration_); }

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;

 private:
  DefaultConfigurationConstraint(const DefaultConfigurationConstraint& other) = default;

  size_t armInputDim_;
  vector_t defaultConfiguration_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2

