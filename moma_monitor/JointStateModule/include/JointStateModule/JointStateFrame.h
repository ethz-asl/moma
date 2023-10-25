//
// Created by acey on 24.10.23.
//

#ifndef MOMA_MONITOR_JOINTSTATEFRAME_H
#define MOMA_MONITOR_JOINTSTATEFRAME_H

#include "JointStateModule.h"
#include "JointStateWidget.h"
#include <state_monitor_sdk/module/ModuleFrame.h>

class JointStateFrame : public ModuleFrame {
public:
  JointStateFrame(IModulePanel *panel, std::shared_ptr<JointStateModule> controller);

protected:
  void timerEvent(QTimerEvent *event) override;
  void showEvent(QShowEvent *event) override;

public:
  ~JointStateFrame() override = default;

private:
  std::vector<JointStateWidget*> joint_state_widgets_;
  std::shared_ptr<JointStateModule> joint_state_module_;
  void updateEvent();
  [[nodiscard]] int getDialCount() const;
};

#endif // MOMA_MONITOR_JOINTSTATEFRAME_H
