//
// Created by giuseppe on 25.01.21.
//
#include "moma_sensor_tools/ft_sensor_utils.h"

#include <iostream>

#include "moma_sensor_tools/parser_utilts.h"

namespace moma_sensor_tools {

bool get_ft_calibration_from_file(const std::string& file_path, FTSensorCalibrationData& data) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException& ex) {
    std::cout << ex.what() << std::endl;
  } catch (const YAML::BadFile& ex) {
    std::cout << ex.what() << std::endl;
  }

  YAML::Node calibration_data = config["calibration_data"];
  if (!calibration_data) {
    std::cout << "No calibration_data found\n";
    return false;
  }

  if (!calibration_data["mass"]) {
    std::cout << "Could not find entry: mass" << std::endl;
    return false;
  }
  data.mass = calibration_data["mass"].as<double>();

  if (!calibration_data["mass"]) {
    std::cout << "Could not find entry: mass" << std::endl;
    return false;
  }

  bool ok = true;
  ok &= moma_sensor_tools::parse_key(calibration_data, "mass", data.mass);
  ok &= moma_sensor_tools::parse_vector(calibration_data, "com", data.com);
  ok &= moma_sensor_tools::parse_vector(calibration_data, "bias", data.bias);
  return ok;
}
}  // namespace moma_sensor_tools

std::ostream& operator<<(std::ostream& os, const moma_sensor_tools::Wrench& wrench) {
  os << "Force: " << wrench.get_force().transpose()
     << ", Torque: " << wrench.get_torque().transpose();
  return os;
}

std::ostream& operator<<(std::ostream& os, const moma_sensor_tools::FTSensorCalibrationData& data) {
  os << "\n======================================================\n";
  os << "              FT Sensor Calibration Data              \n";
  os << "======================================================\n";
  os << "Payload mass: " << data.mass << std::endl;
  os << "Payload com:  " << data.com.transpose() << std::endl;
  os << "Force bias:   " << data.get_force_bias().transpose() << std::endl;
  os << "Torque bias:  " << data.get_torque_bias().transpose() << std::endl;
  os << "======================================================\n";
  return os;
}
