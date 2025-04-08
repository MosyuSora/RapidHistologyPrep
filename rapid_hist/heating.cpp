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
}

void tempSetInit() {
  if(targetTemp == 0.0){
    targetTemp == 40.0;
    phase = Phase::HEAT_40;
  }

  switch (phase) {
    case Phase::HEAT_40:
      targetTemp = 40.0;
      tempTol = 0.4;
      break;
    case Phase::HEAT_60:
      targetTemp = 60.0;
      tempTol = 1.0;
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
  currentTemp = readTemperature(1000);
  heatPwr = HeatPID.getHeatPwr();

  if (currentTemp == -999) return;

  if (abs(currentTemp - targetTemp) <= tempTol && tempSetpointCnt < 128) {
    tempSetpointCnt++;
  } else {
    tempSetpointCnt = 0;
  }

  int dutyRatioInt = HeatPID.compute(currentTemp);
  analogWrite(HEAT_PIN, dutyRatioInt);

  switch (phase) {
    case Phase::HEAT_40:
      if (tempSetpointCnt >= 10) {
        phase = Phase::SOAK_40;
        phaseStartTime = millis();
        tempSetpointCnt = 0;
        startFluidCycle();
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
      if (millis() - phaseStartTime >= 0.5 * 60000) {  // 30 秒
        phase = Phase::COOL_40;
        tempSetpointCnt = 0;
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

