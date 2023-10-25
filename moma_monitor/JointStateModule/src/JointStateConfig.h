//
// Created by acey on 24.10.23.
//

#ifndef MOMA_MONITOR_JOINTSTATECONFIG_H
#define MOMA_MONITOR_JOINTSTATECONFIG_H

#include <state_monitor_sdk/Config.h>

class JointStateConfig : public Config {

public:
  explicit JointStateConfig(std::function<void()> fn);
  void reload() override;
  void load(const YAML::Node &node) override;
  YAML::Node save() override;

  inline static constexpr const char* DIAL_COUNT = "dial_count";
  inline static constexpr const int DIAL_COUNT_DEFAULT = 9;
  inline static constexpr const char* TOPIC = "topic";
};

#endif // MOMA_MONITOR_JOINTSTATECONFIG_H
