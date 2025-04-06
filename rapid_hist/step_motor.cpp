#include "step_motor.h"

stepMotor::stepMotor(int step, int dir, int slp, int delayMicros)
  : stepPin(step), dirPin(dir), slpPin(slp), stepDelay(delayMicros),
    lastStepTime(0), currentStep(0), totalSteps(0), stepHigh(false), running(false) {}

void stepMotor::start(int steps, bool forward) {
  totalSteps = steps;
  currentStep = 0;
  running = true;
  digitalWrite(dirPin, forward ? LOW : HIGH);
  digitalWrite(slpPin, HIGH);
}

void stepMotor::update() {
  if (!running) return;
  unsigned long now = micros();
  if (now - lastStepTime >= stepDelay) {
    lastStepTime = now;
    digitalWrite(stepPin, stepHigh ? HIGH : LOW);
    stepHigh = !stepHigh;
    if (!stepHigh) {
      currentStep++;
      if (currentStep >= totalSteps) {
        running = false;
        digitalWrite(slpPin, LOW);
      }
    }
  }
}

bool stepMotor::isIdle() {
  return !running;
}
