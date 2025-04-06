#include "temperature.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

medianFilter::medianFilter() : index(0) {
  for (int i = 0; i < FILTER_SIZE; i++) {
    data[i] = 0;
  }
}

int medianFilter::filter(int samples[]) {
  int sortedData[FILTER_SIZE];
  memcpy(sortedData, samples, sizeof(int) * FILTER_SIZE);
  for (int i = 0; i < FILTER_SIZE - 1; i++) {
    for (int j = i + 1; j < FILTER_SIZE; j++) {
      if (sortedData[i] > sortedData[j]) {
        int temp = sortedData[i];
        sortedData[i] = sortedData[j];
        sortedData[j] = temp;
      }
    }
  }
  return sortedData[FILTER_SIZE / 2];
}
float temperatureFromResistance(float R) {
  float lnR = log(R);
  return
    -0.410963 * pow(lnR, 5) +
    23.007500 * pow(lnR, 4) +
    -511.787183 * pow(lnR, 3) +
    5660.766498 * pow(lnR, 2) +
    -31182.521670 * lnR +
    68609.215924;
}
float readTemperature(int sampleInterval) {
  static enum { IDLE, SAMPLING, CALCULATING } state = IDLE;
  static unsigned long lastSampleTime = 0;
  static int adcIndex = 0;
  static int adcReadings[FILTER_SIZE];

  const float knownResistor = 9840.0;
  const float VSupply = 5.0;

  switch (state) {
    case IDLE:
      if (millis() - lastSampleTime >= sampleInterval) {
        state = SAMPLING;
        adcIndex = 0;
        memset(adcReadings, 0, sizeof(adcReadings));
      }
      break;

    case SAMPLING: {
      static unsigned long lastADC = 0;
      if (adcIndex < FILTER_SIZE && millis() - lastADC >= 2) {
        adcReadings[adcIndex++] = analogRead(THERMISTOR_PIN);
        lastADC = millis();
      } else if (adcIndex >= FILTER_SIZE) {
        state = CALCULATING;
      }
      break;
    }

    case CALCULATING: {
      medianFilter filter;
      int adcVal = filter.filter(adcReadings);
      float readVolt = (adcVal / 1023.0) * VSupply;
      float thermistorR = knownResistor * (readVolt / (VSupply - readVolt));
      float temp = temperatureFromResistance(thermistorR);
      state = IDLE;
      lastSampleTime = millis();
      return temp;
    }
  }

}
