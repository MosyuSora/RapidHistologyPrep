#ifndef SERIAL_LCD_H
#define SERIAL_LCD_H

#include <Arduino.h>
#include "config.h"

class serialLCD {
  private:
    String reagentStage;
    String formInfo;
    String pumpInfo;
    float heatPWR;
    int nowTemp;
    int setTemp;
    unsigned long lastUpdateTime;
    int updateInterval;

  public:
    serialLCD();

    void setReagentStage(const String& stage);
    void setFormInfo(const String& info);
    void setPumpInfo(const String& info);
    void setHeatPower(float power);
    void setNowTemp(int temp);
    void setSetTemp(int temp);

    void updateFrame();
    void setPage(const String& pageName);
};

namespace lcdPara {
  extern serialLCD lcd;
}

#endif
