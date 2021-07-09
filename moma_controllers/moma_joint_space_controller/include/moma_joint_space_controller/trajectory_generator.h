#pragma once

// adapted from
// https://github.com/PR2/pr2_common_actions/blob/kinetic-devel/joint_trajectory_generator/include/joint_trajectory_generator/trajectory_generation.h

#include <moma_joint_space_controller/velocity_profile.h>
#include <vector>
#include <Eigen/Core>
#include_next <kdl/velocityprofile_trap.hpp>

namespace moma_controllers{

class TrajectoryGenerator
{
 public:
  TrajectoryGenerator(): TrajectoryGenerator(1.0, 10.0, 0){};
  TrajectoryGenerator(double max_vel, double max_acc, unsigned int size);
  ~TrajectoryGenerator();

  void compute(const Eigen::VectorXd& start,
               const Eigen::VectorXd& end,
               const double t_start);


  Eigen::VectorXd get_next_point(const double time);

 private:
  double max_time;
  double initial_time;
 public:
  std::vector<std::unique_ptr<VelocityProfile_Trap>> generators_;
};

}

std::ostream& operator<<(std::ostream& os, const moma_controllers::TrajectoryGenerator& gen);