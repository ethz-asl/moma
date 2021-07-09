#include "moma_joint_space_controller/trajectory_generator.h"
#include <iostream>

namespace moma_controllers{

TrajectoryGenerator::TrajectoryGenerator(double max_vel, double max_acc, unsigned int size)
{
  for (unsigned int i=0; i<size; i++){
    generators_.push_back(std::make_unique<VelocityProfile_Trap>(max_vel, max_acc));
  }
}

TrajectoryGenerator::~TrajectoryGenerator(){}


void TrajectoryGenerator::compute(const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& end,
                                  double t_start) {

  initial_time = t_start;

  // check
  if (start.size() != generators_.size() ||
      end.size() != generators_.size()) {
    throw std::runtime_error("Point size different from generator size.");
  }

  // generate initial profiles
  for (unsigned int i = 0; i < generators_.size(); i++){
    generators_[i]->SetProfile(start(i), end(i));
  }

  // find profile that takes most time
  max_time = -1.0;
  for (unsigned int i = 0; i < generators_.size(); i++)
    if (generators_[i]->Duration() > max_time)
      max_time = generators_[i]->Duration();

  max_time += 0.001;
  // generate profiles with max time
  for (unsigned int i = 0; i < generators_.size(); i++)
    generators_[i]->SetProfileDuration(start(i), end(i), max_time);
}


Eigen::VectorXd TrajectoryGenerator::get_next_point(const double time){
  Eigen::VectorXd q_out(generators_.size());
  double t = time - initial_time;
  t = (t > max_time) ? max_time : t;
  for (unsigned int i = 0; i < generators_.size(); i++) {
    q_out(i) = generators_[i]->Pos(t);
    //qd_out(i) =  generators_[i]->Vel(t);
    //qdd_out(i) = generators_[i]->Acc(t);
  }
  return q_out;
}

}

std::ostream& operator<<(std::ostream& os, const moma_controllers::TrajectoryGenerator& gen){
  for(const auto& g : gen.generators_){
    os << std::endl;
  }
  return os;
}
