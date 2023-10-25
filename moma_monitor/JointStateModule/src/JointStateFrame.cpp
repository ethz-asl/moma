//
// Created by acey on 24.10.23.
//

#include "JointStateFrame.h"
#include "JointStateConfig.h"

#include <QLabel>
#include <utility>

JointStateFrame::JointStateFrame(IModulePanel *panel, std::shared_ptr<JointStateModule> controller)
    : ModuleFrame(panel, controller) {
  joint_state_module_ = std::move(controller);
  joint_state_module_->setConfigChangeCallback([this]() {
    updateEvent();
  });
  startTimer(16); // 60fps
}

void JointStateFrame::showEvent(QShowEvent *event) {
  getModule()->getConfig()->reload();
}

void JointStateFrame::timerEvent(QTimerEvent *event) {
  if (joint_state_module_->hasUpdate()) {
    if (joint_state_widgets_.size() != getDialCount()) {
      return;
    }
    std::vector<double> positions = joint_state_module_->getPositions();
      for (int i = 0; i < getDialCount(); i++) {
        if (i < positions.size()) {
          joint_state_widgets_.at(i)->setCurrent(positions.at(i));
        } else {
          joint_state_widgets_.at(i)->setCurrent(0);
        }
        joint_state_widgets_.at(i)->update();
      }
    return;
  }
}


void JointStateFrame::updateEvent() {
  for (const auto& dial: joint_state_widgets_) {
    delete dial;
  }
  joint_state_widgets_.clear();

  for (int i = 0; i < getDialCount(); i++) {
    auto *dial = new JointStateWidget();
    double min = getModule()->getConfig()->getValue<double>( "dial_min_" + std::to_string(i));
    double max = getModule()->getConfig()->getValue<double>( "dial_max_" + std::to_string(i));
    dial->setRange(min, max);
    dial->setCurrent(0);
    addWidget(dial);
    joint_state_widgets_.emplace_back(dial);
  }

  update();
}

int JointStateFrame::getDialCount() const {
  return joint_state_module_->getConfig()->getValue<int>(JointStateConfig::DIAL_COUNT);
}
