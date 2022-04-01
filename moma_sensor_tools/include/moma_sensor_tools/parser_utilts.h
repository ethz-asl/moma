//
// Created by giuseppe on 25.01.21.
//

#pragma once
#include <yaml-cpp/yaml.h>

#include <iostream>

namespace moma_sensor_tools {

template <typename T>
bool parse_key(const YAML::Node& node, const std::string& key, T& value) {
  if (!node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    return false;
  }
  value = node[key].as<T>();
  return true;
}

template <int N>
bool parse_vector(const YAML::Node& node, const std::string& key,
                  Eigen::Matrix<double, N, 1>& value) {
  if (!node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    return false;
  }

  if (!node[key].IsSequence()) {
    std::cout << "Failed to parse vector: node is not a sequence!" << std::endl;
    return false;
  }
  if (node[key].size() != N) {
    std::cout << "Failed to parse vector: size is inconsistent!" << std::endl;
    return false;
  }

  for (size_t i = 0; i < N; i++) {
    value[i] = node[key][i].as<double>();
  }
  return true;
}

}  // namespace moma_sensor_tools
