//
// Created by acey on 24.10.23.
//

#ifndef MOMA_MONITOR_JOINTSTATEMODULE_H
#define MOMA_MONITOR_JOINTSTATEMODULE_H

#include "state_monitor_sdk/api/IOMavMessageController.h"
#include <state_monitor_sdk/module/Module.h>
#include <state_monitor_sdk/module/ModuleInfo.h>
#include "sensor_msgs/JointState.h"

using namespace sensor_msgs;

class JointStateModule : public Module {
public:
  JointStateModule();
  void configChangeEvent() override;
  void update(const JointState &msg);
  void setConfigChangeCallback(std::function<void()> fn);
  inline static constexpr const char *NAME = "JointState";
  inline static const ModuleVersion VERSION{1, 0, 0};
  [[nodiscard]] const std::atomic<bool> &hasUpdate() const;
  std::vector<double> getPositions();
  ~JointStateModule() override;

private:
  std::function<void(const JointState &msg)> joint_state_callback_;
  std::vector<double> positions_;
  std::atomic<bool> has_update_{false};
  std::function<void()> frame_update_fn_;
  IOMavMessageController *msg_controller_ = IOMavMessageController::get();
  std::string old_key_{};
};

#endif // MOMA_MONITOR_JOINTSTATEMODULE_H
