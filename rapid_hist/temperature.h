#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "config.h"

class medianFilter {
  private:
    int data[FILTER_SIZE];
    int index;
  public:
    medianFilter();
    int filter(int samples[]);
};

float readTemperature(int sampleInterval = 1000);
float temperatureFromResistance(float R);
#endif
