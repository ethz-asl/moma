//
// Created by giuseppe on 25.01.21.
//

#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace moma_sensor_tools {

struct Wrench {
  Wrench() {
    force.setZero();
    torque.setZero();
  }
  Wrench(const Eigen::Vector3d& f, const Eigen::Vector3d& t) : force(f), torque(t){};
  Wrench(const Wrench& rhs) {
    force = rhs.force;
    torque = rhs.torque;
  }
  Wrench(const Eigen::Matrix<double, 6, 1>& rhs){
    force = rhs.head<3>();
    torque = rhs.tail<3>();
  }

  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  Eigen::Matrix<double, 6, 1>& to_vector() { vec << force, torque; return vec; }
  [[nodiscard]] inline const Eigen::Vector3d& get_force() const { return force; }
  [[nodiscard]] inline const Eigen::Vector3d& get_torque() const { return torque; }
  inline Eigen::Vector3d& get_force() { return force; }
  inline Eigen::Vector3d& get_torque() { return torque; }

 private:
  Eigen::Matrix<double, 6, 1> vec;
};

struct FTSensorCalibrationData {
  FTSensorCalibrationData() : mass(0.0) {
    com.setZero();
    bias.setZero();
  }

  double mass;
  Eigen::Matrix<double, 6, 1> bias;
  Eigen::Vector3d com;

  inline const Eigen::Matrix<double, 6, 1>& get_bias() { return bias; }
  inline void set_force_bias(const Eigen::Vector3d& fb) { bias.head(3) = fb; }

  inline void set_torque_bias(const Eigen::Vector3d& tb) { bias.tail(3) = tb; }

  inline const Eigen::Vector3d get_force_bias() const { return bias.head(3); }
  inline const Eigen::Vector3d get_torque_bias() const { return bias.tail(3); }
};

bool get_ft_calibration_from_file(const std::string& file_path, FTSensorCalibrationData&);

}  // namespace sensor_tools::ft

std::ostream& operator<<(std::ostream& os, const moma_sensor_tools::Wrench&);

std::ostream& operator<<(std::ostream& os, const moma_sensor_tools::FTSensorCalibrationData& data);