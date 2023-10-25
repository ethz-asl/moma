//
// Created by acey on 24.10.23.
//

#include "JointStateConfig.h"

#include <utility>

JointStateConfig::JointStateConfig(std::function<void()> fn)
    : Config(std::move(fn)){
  setValue<int>(DIAL_COUNT, DIAL_COUNT_DEFAULT);

  setValue<std::string>(TOPIC, "joint_state");
  for (int i = 0; i < getValue<int>(DIAL_COUNT); ++i) {
    std::string min_key = "dial_min_" + std::to_string(i);
    std::string max_key = "dial_max_" + std::to_string(i);
    setValue<double>(min_key, 0.0);
    setValue<double>(max_key, 1.0);

  }
}

void JointStateConfig::reload() {
  auto num_dials = getValue<int>(DIAL_COUNT);
  for (int i = 0; i < num_dials; ++i) {
    std::string min_key = "dial_min_" + std::to_string(i);
    std::string max_key = "dial_max_" + std::to_string(i);
    setValue(min_key, getOptValue<double>(min_key).value_or(0.0));
    setValue(max_key, getOptValue<double>(max_key).value_or(1.0));
  }
  config_change_callback_();
}

void JointStateConfig::load(const YAML::Node &node) {
  setValue<int>(DIAL_COUNT, node[DIAL_COUNT].as<int>(DIAL_COUNT_DEFAULT));
  setValue<std::string>(TOPIC, node[TOPIC].as<std::string>("joint_state"));

  for (int i = 0; i < getValue<int>(DIAL_COUNT); ++i) {
    std::string min_key = "dial_min_" + std::to_string(i);
    std::string max_key = "dial_max_" + std::to_string(i);
    setValue<double>(min_key, node[min_key].as<double>(0.0));
    setValue<double>(max_key, node[max_key].as<double>(1.0));
  }
  config_change_callback_();
}

YAML::Node JointStateConfig::save() {
  YAML::Node node{};
  node[DIAL_COUNT] = getValue<int>(DIAL_COUNT);
  node[TOPIC] = getValue<std::string>(TOPIC);
  for (int i = 0; i < getValue<int>(DIAL_COUNT); i++) {
    std::string min_key = "dial_min_" + std::to_string(i);
    std::string max_key = "dial_max_" + std::to_string(i);
    node[min_key] = getValue<double>(min_key);
    node[max_key] = getValue<double>(max_key);
  }
  return node;
}
