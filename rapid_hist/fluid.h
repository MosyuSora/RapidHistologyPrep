#ifndef FLUID_H
#define FLUID_H

#include "step_motor.h"
#include "config.h"
#include "recipe.h"
#include "heating.h"

namespace fluidPara {

  enum class FluidState {
    IDLE, INIT_PUMP, PUMP_IN, BACKFLOW, SOAKING, PUMP_OUT, WAIT_CYCLE
  };

  extern FluidState state;

  extern int cycleIndex;
  extern int steps;
  extern bool backflow;
  extern unsigned long lastTime;
  extern unsigned long soakStartTime;

  extern stepMotor motorInlet;
  extern stepMotor motorOutlet;

  extern recipeManager recipe;
}
  int calculateSteps(float volume);
  void startFluidCycle();
  void processFluid();
  void motorControl();


#endif
