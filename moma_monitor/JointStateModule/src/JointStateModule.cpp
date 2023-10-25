//
// Created by acey on 24.10.23.
//

#include "JointStateModule.h"

#include <utility>
#include "JointStateConfig.h"

JointStateModule::JointStateModule() : Module("JointState", new JointStateConfig([this](){
                                                configChangeEvent();})) {
  joint_state_callback_ = [this](auto &&PH1) {
    update(std::forward<decltype(PH1)>(PH1));
  };
}

void JointStateModule::setConfigChangeCallback(std::function<void()> fn) {
  frame_update_fn_ = std::move(fn);
}

void JointStateModule::update(const JointState &msg) {
  positions_ = msg.position;
  has_update_ = true;
}

const std::atomic<bool> &JointStateModule::hasUpdate() const {
  return has_update_;
}

std::vector<double> JointStateModule::getPositions() {
  return positions_;
}

void JointStateModule::configChangeEvent() {
  if (frame_update_fn_) {
    frame_update_fn_();
  }

  if (!old_key_.empty()) {
    msg_controller_->removeSubscriber(old_key_);
  }

  std::string topic_name = getConfig()->getValue<std::string>(JointStateConfig::TOPIC);
  std::string key = NAME + std::string("/") + topic_name;
  msg_controller_->addSubscriber(key, new DynamicSubscriber(topic_name, joint_state_callback_));
  old_key_ = key;
}

JointStateModule::~JointStateModule() {
  msg_controller_->removeSubscriber(old_key_);
}
