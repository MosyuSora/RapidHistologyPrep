#ifndef SYSTEM_CONTROL_H
#define SYSTEM_CONTROL_H

#include "config.h"

// 按钮控制
void handleButton();

// LED 状态更新
void updateLEDStatus();

// 系统控制行为
void emergencyStop();
void startSystem();

// 状态变量（需要由外部访问）
namespace buttonPara {
  extern bool lastButtonState;
}

namespace ledPara {
  extern unsigned long lastBlinkTime;
  extern int blinkInterval;
  extern bool ledState;
}

#endif
