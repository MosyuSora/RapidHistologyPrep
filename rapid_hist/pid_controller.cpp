#include "pid_controller.h"
#include <Arduino.h>

pidController::pidController(float kp, float ki, float kd, float target)
  : Kp(kp), Ki(ki), Kd(kd), setpoint(target), sumError(0), lastError(0), dutyRatio(0) {}

void pidController::setTarget(float target) {
  setpoint = target;
  sumError = 0;
  lastError = 0;
}

int pidController::compute(float temperature) {
  float error = setpoint - temperature;
  float P_out = Kp * error;
  float maxIntegral = 150;
  if (abs(error) < 5.0) {
    sumError += error;
    sumError = constrain(sumError, -maxIntegral, maxIntegral);
  }
  float I_out = Ki * sumError;
  float derivative = error - lastError;
  float D_out = Kd * derivative;
  float output = constrain(P_out + I_out + D_out, 0, 255);
  lastError = error;
  dutyRatio = 100 * output / 255;
  return (int)output;
}

float pidController::getHeatPwr() {
  return dutyRatio;
}
