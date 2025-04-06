#include "system_control.h"
#include "heating.h"
#include "fluid.h"
#include "serial_lcd.h"

using namespace heatingPara;
using namespace fluidPara;

namespace buttonPara {
  bool lastButtonState = HIGH;
}

namespace ledPara {
  unsigned long lastBlinkTime = 0;
  int blinkInterval = 32767;
  bool ledState = LOW;
}

void emergencyStop() {
  phase = Phase::IDLE;
  state = FluidState::IDLE;
  ledPara::blinkInterval = 32760;
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void startSystem() {
  tempSetInit();
  lcdPara::lcd.setPage("menu_run");
  digitalWrite(LED_BLUE_PIN, LOW);
}

void updateLEDStatus() {
  unsigned long now = millis();
  unsigned long halfInterval = ledPara::blinkInterval / 2;

  if (now - ledPara::lastBlinkTime >= halfInterval) {
    ledPara::lastBlinkTime = now;
    ledPara::ledState = !ledPara::ledState;
    digitalWrite(LED_RED_PIN, ledPara::ledState);
  }

  if (heatingPara::tempSetpointCnt >= 10) {
    digitalWrite(LED_BLUE_PIN, HIGH);
  }

  // Green LED stays ON during setup()
}

void handleButton() {
  static bool isPressing = false;
  static unsigned long pressStartTime = 0;
  bool reading = digitalRead(BOARD_BTN); // Active LOW

  if (reading == LOW && buttonPara::lastButtonState == HIGH) {
    isPressing = true;
    pressStartTime = millis();
  }

  if (isPressing && reading == LOW) {
    if (millis() - pressStartTime >= 5000) {
      emergencyStop(); // 5s hold
      isPressing = false;
    }
  }

  if (reading == HIGH && buttonPara::lastButtonState == LOW) {
    isPressing = false;
  }

  buttonPara::lastButtonState = reading;
}
