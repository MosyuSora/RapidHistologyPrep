#include "fluid.h"
#include "heating.h"  
using namespace fluidPara;


namespace fluidPara {
  // Constants for fluid volume calculation
    const double pi = M_PI;
    const double ID = 0.4;                      // Tubing ID in cm
    const double TLengthin = 25.0;              // Tubing Length in cm for input
    const double TLengthout = 25.0;             // Tubing Length in cm for output
    const double flow = 3.0/5.0;                // Volumetric flow rate in mL/rotation

  // Fluid control state machine definition
  // System idle
  // Initial pump preparation
  // Pumping fluid into the system
  // Reversing flow for backwash
  // Soaking phase for fluid interaction
  // Pumping fluid out of the system
  // Waiting between cycles


    int cycleIndex = 0;                          // Index to track the current cycle
    int steps = 0;                               // Steps required for motor to move specific volume
    bool backflow = false;                       // Indicator if backflow is required in the current cycle
    unsigned long lastTime = 0;                  // Timestamp for tracking delays in processes
    unsigned long soakStartTime = 0;             // Timestamp when soaking starts
    recipeManager recipe = recipeManager();
    FluidState state = FluidState::IDLE;
  // Stepper motor objects for inlet and outlet fluid control
    stepMotor motorInlet(STEP_PIN_1, DIR_PIN_1, SLP_PIN_1, 1750);
    stepMotor motorOutlet(STEP_PIN_2, DIR_PIN_2, SLP_PIN_2, 1750);
}

int calculateSteps(float volume) {
  return ceil(volume / flow) * STEPS_PER_REV;
}

void startFluidCycle() {
  if (state == FluidState::IDLE) {
    state = FluidState::INIT_PUMP;
    cycleIndex = 0;
    lastTime = millis();
  }
}

void processFluid() {
  unsigned long now = millis();

  switch (state) {
    case FluidState::IDLE:
      return;

    case FluidState::INIT_PUMP:
      steps = (cycleIndex == 0 || cycleIndex == 1 || cycleIndex == 3 || cycleIndex == 6)
              ? calculateSteps(recipe.recipe[cycleIndex].volume)
              : calculateSteps(recipe.recipe[cycleIndex].volume+recipe.recipe[cycleIndex].deadVolume);
      backflow = (cycleIndex == 0 || cycleIndex == 1 || cycleIndex == 3 || cycleIndex == 6);
      motorInlet.start(steps, false);
      state = FluidState::PUMP_IN;
      return;

    case FluidState::PUMP_IN:
      if (motorInlet.isIdle()) {
        if (backflow) {
          state = FluidState::BACKFLOW;
          motorInlet.start(steps, true);
        } else {
          state = FluidState::SOAKING;
          soakStartTime = millis();
        }
      }
      return;

    case FluidState::BACKFLOW:
      if (motorInlet.isIdle()) {
        state = FluidState::SOAKING;
        soakStartTime = millis();
      }
      return;

    case FluidState::SOAKING:
      if (now - soakStartTime >= recipe.recipe[cycleIndex].time * 60000) {
        state = FluidState::PUMP_OUT;
        motorOutlet.start(calculateSteps(15.0), true);
      }
      return;

    case FluidState::PUMP_OUT:
      if (motorOutlet.isIdle()) {
        state = FluidState::WAIT_CYCLE;
        lastTime = millis();
      }
      return;

    case FluidState::WAIT_CYCLE:
      if (now - lastTime >= 5000) {
        cycleIndex++;
        if (cycleIndex < 8) {
          state = FluidState::INIT_PUMP;
        } else {
          state = FluidState::IDLE;
          heatingPara::phase = heatingPara::Phase::HEAT_60;
          tempSetInit();
        }
      }
      return;
  }
}

void motorControl() {
  motorInlet.update();
  motorOutlet.update();
}
