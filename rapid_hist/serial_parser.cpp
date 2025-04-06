#include "serial_parser.h"
#include "fluid.h" // 为了操作 recipe
#include "system_control.h"
#include <string.h>

using namespace fluidPara;

serialFrameParser::serialFrameParser() : state(WAIT_HEADER1) {}

void serialFrameParser::reset() {
  state = WAIT_HEADER1;
  memset(recipeNameBuffer, 0, sizeof(recipeNameBuffer));
  recipeNameIndex = 0;
  currentStep = 0;
  dataByteCounter = 0;
}

void serialFrameParser::processRecipe() {
  strncpy(recipe.recipeName, recipeNameBuffer, 15);
  recipe.recipeName[15] = '\0';
  for (int i = 0; i < 8; i++) {
    recipe.recipe[i].volume = volumes[i] / 100.0f;
    recipe.recipe[i].time = times[i] / 100.0f;
  }
}

void serialFrameParser::parseByte(uint8_t byte) {
  switch (state) {
    case WAIT_HEADER1:
      if (byte == 0xAA) state = WAIT_HEADER2;
      break;

    case WAIT_HEADER2:
      if (byte == 0xBB) {
        cmdType = START_CMD;
        state = CMD_END_CHECK;
        endMarkerCount = 2;
      } else if (byte == 0xCC) {
        cmdType = STOP_CMD;
        state = CMD_END_CHECK;
        endMarkerCount = 2;
      } else if (byte == 0xAA) {
        cmdType = RECIPE_CMD;
        state = READ_RECIPE_NAME;
      } else {
        reset();
      }
      break;

    case CMD_END_CHECK:
      if (byte == 0xFF && --endMarkerCount == 0) {
        if (cmdType == START_CMD) startSystem();
        else if (cmdType == STOP_CMD) emergencyStop();
        reset();
      } else if (byte != 0xFF) {
        reset();
      }
      break;

    case READ_RECIPE_NAME:
      if (recipeNameIndex < 15) {
        recipeNameBuffer[recipeNameIndex++] = byte;
        if (byte == 0x00) state = READ_RECIPE_DATA;
      } else {
        recipeNameBuffer[15] = byte;
        state = (byte == 0x00) ? READ_RECIPE_DATA : WAIT_HEADER1;
      }
      break;

    case READ_RECIPE_DATA:
      switch (dataByteCounter % 4) {
        case 0: rawVolume = byte; break;
        case 1: rawVolume |= byte << 8; break;
        case 2: rawTime = byte; break;
        case 3:
          rawTime |= byte << 8;
          if (currentStep < 8) {
            volumes[currentStep] = rawVolume;
            times[currentStep] = rawTime;
            currentStep++;
          }
          break;
      }
      dataByteCounter++;
      if (currentStep >= 8 && dataByteCounter >= 32) {
        state = CHECK_END_MARKER;
        endMarkerCount = 4;
      }
      break;

    case CHECK_END_MARKER:
      if (byte == 0xFF && --endMarkerCount == 0) {
        processRecipe();
        reset();
      } else if (byte != 0xFF) {
        reset();
      }
      break;
  }
}
