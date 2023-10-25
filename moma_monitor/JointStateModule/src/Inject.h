//
// Created by acey on 17.10.23.
//

#ifndef MOMA_MONITOR_INJECT_H
#define MOMA_MONITOR_INJECT_H

#include <state_monitor_sdk/Logging.h>
#include <state_monitor_sdk/SdkConfig.h>
#include <state_monitor_sdk/module/IModulePanel.h>
#include <state_monitor_sdk/module/ModuleFrame.h>

#include "JointStateFrame.h"

STATE_MONITOR_EXPORT ModuleInfo* loadModule() {

  ModuleCreateFn createFn = [](IModulePanel* panel) {
    ModuleFrame* frame = new JointStateFrame(panel, std::make_shared<JointStateModule>());
    Logger::log(I, frame->getModule()->getName(), "Loaded JointStateFrame");
    return frame;
  };

  return new ModuleInfo(JointStateModule::NAME, JointStateModule::VERSION, createFn);
}

#endif // MOMA_MONITOR_INJECT_H
