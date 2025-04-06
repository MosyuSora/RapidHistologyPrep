#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <Arduino.h>

class stepMotor {
  private:
    int stepPin, dirPin, slpPin;
    int stepDelay;
    unsigned long lastStepTime;
    int currentStep, totalSteps;
    bool stepHigh;
    bool running;

  public:
    stepMotor(int step, int dir, int slp, int delayMicros);
    void start(int steps, bool forward);
    void update();
    bool isIdle();
};

#endif
