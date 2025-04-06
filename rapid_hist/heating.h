#ifndef HEATING_H
#define HEATING_H
#define K_P 15
#define K_I 0.02
#define K_D 0.5

#include "pid_controller.h"
#include "temperature.h"
#include "config.h"

namespace heatingPara {

  extern float targetTemp;
  extern float currentTemp;
  extern float heatPwr;
  extern float tempTol;
  extern uint16_t tempSetpointCnt;
  extern pidController HeatPID;
  
   // Enumeration for the different phases of the heating process
  enum class Phase { IDLE, HEAT_40, SOAK_40, HEAT_60, SOAK_60, COOL_40 };
  extern Phase phase;
  extern unsigned long phaseStartTime;

}


void tempSetInit();
void heatingControl();

// external function
void startFluidCycle();

#endif
