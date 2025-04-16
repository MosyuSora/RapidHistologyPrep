#include "heating.h"
#include "temperature.h"
#include "config.h"
#include "serial_lcd.h"
#include "system_control.h"
#include "fluid.h"
using namespace heatingPara;
using namespace ledPara;

namespace heatingPara{
  float targetTemp = 0.0;                      // Initial target temperature in degrees Celsius
  float tempTol = 0.4;
  float currentTemp = 0.0;
  uint16_t tempSetpointCnt = 0;                 // Counter to determine if the target temperature is reached
  pidController HeatPID =pidController(K_P, K_I, K_D, targetTemp);   // Instantiate the PID controller with initial parameters
  unsigned long phaseStartTime = 0; // Initialize to IDLE phase
  float heatPwr = HeatPID.getHeatPwr();
  Phase phase = Phase::IDLE;
  unsigned long lastToggle = 0;
  bool pwmState = false;
}

void lowFreqPWM(int pin, int dutyRatio) {
  using namespace heatingPara;
  using namespace ledPara;
  const int period = 100; 
  unsigned long currentMillis = millis();

  int onTime = period * dutyRatio / 255;
  int offTime = period - onTime;

  static int currentState = LOW;
  static unsigned long interval = onTime;

  if (currentMillis - lastToggle >= interval) {
    lastToggle = currentMillis;

    if (currentState == HIGH) {
      currentState = LOW;
      interval = offTime;
    } else {
      currentState = HIGH;
      interval = onTime;
    }

    digitalWrite(pin, currentState);
  }
}
void tempSetInit() {
  using namespace heatingPara;
  using namespace ledPara;
  if(phase == Phase::IDLE){
    targetTemp = 40.0;
    phase = Phase::HEAT_40;
  }

  switch (phase) {
    case Phase::HEAT_40:
      targetTemp = 40.0;
      tempTol = 0.4;
      break;
    case Phase::HEAT_60:
      targetTemp = 60.0;
      tempTol = 1.5;
      break;
    case Phase::COOL_40:
      targetTemp = 40.0;
      tempTol = 0.4;
      break;
    default:
      break;
  }

  HeatPID.setTarget(targetTemp);
  tempSetpointCnt = 0;
  blinkInterval = (targetTemp == 40.0) ? 1000 : 200;
}

void heatingControl() {
  using namespace heatingPara;
  using namespace ledPara;
  currentTemp = readTemperature(1000);
  heatPwr = HeatPID.getHeatPwr();

  if (abs(currentTemp - targetTemp) <= tempTol && tempSetpointCnt < 128) {
    tempSetpointCnt++;
  }

  int dutyRatioInt = HeatPID.compute(currentTemp);

  lowFreqPWM(HEAT_PIN, dutyRatioInt);

  switch (phase) {
    case Phase::IDLE:
      break;
    case Phase::HEAT_40:
      if (tempSetpointCnt >= 10) {
        phase = Phase::SOAK_40;
        phaseStartTime = millis();
        startFluidCycle();
        tempSetpointCnt = 0;
      }
      break;

    case Phase::SOAK_40:
      break;

    case Phase::HEAT_60:
      if (tempSetpointCnt >= 10) {
        phase = Phase::SOAK_60;
        phaseStartTime = millis();
        tempSetpointCnt = 0;
      }
      break;

    case Phase::SOAK_60:
      if (millis() - phaseStartTime >= 20 * 60000) {  // 20min
        phase = Phase::COOL_40;
      }
      break;

    case Phase::COOL_40:
      if (tempSetpointCnt >= 10) {
        phase = Phase::IDLE;
        tempSetpointCnt = 0;
      }
      break;

    default:
      break;
  }
}

