#include "config.h"
#include "system_control.h"
#include "heating.h"
#include "fluid.h"
#include "serial_parser.h"
#include "serial_lcd.h"

serialFrameParser parser;

void setup() {
  Serial.begin(9600);
  pinMode(BOARD_BTN, INPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);

  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void loop() {
  if (Serial.available()) {
    parser.parseByte(Serial.read());
  }
  handleButton();
  heatingControl();
  processFluid();
  motorControl();
  updateLEDStatus();
  updateLCD();
}
