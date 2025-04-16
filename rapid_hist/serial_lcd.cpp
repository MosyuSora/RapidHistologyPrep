#include "serial_lcd.h"
#include "heating.h"
#include "fluid.h"
#include "recipe.h"

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

void updateLCD(){
  String pumpInfo;
  switch(fluidPara::state){
    case fluidPara::FluidState::PUMP_IN:
      pumpInfo = "Inlet Pump-in ";
      pumpInfo += String(fluidPara::recipe.recipe[fluidPara::cycleIndex].volume,2);
      pumpInfo += " mL";
      break;

    case fluidPara::FluidState::BACKFLOW:
      pumpInfo = "Inlet Back-flow";
      break;
    case fluidPara::FluidState::PUMP_OUT:
      pumpInfo = "Outlet Pump-out 15 mL";
      break;
    case fluidPara::FluidState::SOAKING:
      pumpInfo = "Soaking now, remaining ";
      pumpInfo += String((ceil(60000*(fluidPara::recipe.recipe[fluidPara::cycleIndex].time))-(millis()-fluidPara::soakStartTime))/1000,0);
      pumpInfo += " s";
      break;
    default:
      pumpInfo = "IDLE";
      if(heatingPara::phase==heatingPara::Phase::HEAT_40){
        pumpInfo +=", wait for 40C heating";
      }
      if(heatingPara::phase==heatingPara::Phase::HEAT_60){
        pumpInfo +=", wait for 60C heating";
      }
      if(heatingPara::phase==heatingPara::Phase::COOL_40){
        pumpInfo +=", wait for 40C cooldown";
      }
      if(heatingPara::phase==heatingPara::Phase::SOAK_60){
        pumpInfo = "Soaking now, remaining ";
        pumpInfo += String((60000*20-(millis()-heatingPara::phaseStartTime))/60000,2);
        pumpInfo += " min";
      }
      break;
  }
  
  lcdPara::lcd.setSetTemp(ceil(heatingPara::targetTemp));
  lcdPara::lcd.setNowTemp(ceil(heatingPara::currentTemp));
  lcdPara::lcd.setHeatPower(heatingPara::HeatPID.getHeatPwr());
  lcdPara::lcd.setFormInfo(fluidPara::recipe.recipeName);
  lcdPara::lcd.setReagentStage(fluidPara::recipe.recipe[fluidPara::cycleIndex].reagentName);
  lcdPara::lcd.setPumpInfo(pumpInfo);
  lcdPara::lcd.updateFrame();
}


namespace lcdPara {
  serialLCD lcd;
}
