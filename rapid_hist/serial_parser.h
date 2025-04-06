#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>

class serialFrameParser {
  public:
    serialFrameParser();
    void parseByte(uint8_t byte);

  private:
    enum ParserState {
      WAIT_HEADER1, WAIT_HEADER2,
      CMD_END_CHECK,
      READ_RECIPE_NAME, READ_RECIPE_DATA,
      CHECK_END_MARKER
    };

    enum CommandType {
      START_CMD, STOP_CMD, RECIPE_CMD
    };

    ParserState state;
    CommandType cmdType;

    uint8_t endMarkerCount;
    uint8_t recipeNameIndex;
    char recipeNameBuffer[16];
    uint8_t currentStep;
    uint8_t dataByteCounter;
    uint16_t rawVolume;
    uint16_t rawTime;
    uint16_t volumes[8];
    uint16_t times[8];

    void reset();
    void processRecipe();
};


void emergencyStop();
void startSystem();

#endif
