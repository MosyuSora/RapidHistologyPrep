#include "serial_lcd.h"

serialLCD::serialLCD()
  : reagentStage("IDLE"), formInfo("Not selected"), pumpInfo("IDLE"),
    heatPWR(0.0), nowTemp(0), setTemp(0), lastUpdateTime(0),
    updateInterval(LCD_UPDATE_INTERVAL) {}

void serialLCD::setReagentStage(const String& stage) { reagentStage = stage; }
void serialLCD::setFormInfo(const String& info) { formInfo = info; }
void serialLCD::setPumpInfo(const String& info) { pumpInfo = info; }
void serialLCD::setHeatPower(float power) { heatPWR = power; }
void serialLCD::setNowTemp(int temp) { nowTemp = temp; }
void serialLCD::setSetTemp(int temp) { setTemp = temp; }

void serialLCD::setPage(const String& pageName) {
  Serial.print("page ");
  Serial.print(pageName);
  Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
}

void serialLCD::updateFrame() {
  unsigned long now = millis();
  if (now - lastUpdateTime >= updateInterval) {
    lastUpdateTime = now;

    Serial.print("r_stage.txt=\""); Serial.print(reagentStage); Serial.print("\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("form.txt=\""); Serial.print(formInfo); Serial.print("\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("pump.txt=\""); Serial.print(pumpInfo); Serial.print("\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("heat.txt=\""); Serial.print(heatPWR, 0); Serial.print("%PWR\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("now_t.txt=\""); Serial.print(nowTemp); Serial.print("\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("set_t.txt=\""); Serial.print(setTemp); Serial.print("\"");
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);

    Serial.print("add 4,0,"); Serial.print(nowTemp);
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
    Serial.print("add 5,0,"); Serial.print(nowTemp);
    Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
  }
}

// 全局 LCD 对象
namespace lcdPara {
  serialLCD lcd;
}
