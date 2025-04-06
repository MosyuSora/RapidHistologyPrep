#ifndef RECIPE_H
#define RECIPE_H

#include <Arduino.h>
#include <string.h>

struct formStep {
  String reagentName;
  float volume;
  float time;
  formStep(const String& name = "", float v = 5.0, float t = 5.0)
    : reagentName(name), volume(v), time(t) {}
};

struct recipeManager {
  char recipeName[16];
  formStep recipe[9];

  recipeManager() {
    strcpy(recipeName, "Default");
    recipe[0].reagentName = "10%Formalin";
    recipe[1].reagentName = "70%Ethanol";
    recipe[2].reagentName = "70%Ethanol";
    recipe[3].reagentName = "95%Ethanol";
    recipe[4].reagentName = "95%Ethanol";
    recipe[5].reagentName = "Xylene";
    recipe[6].reagentName = "Xylene";
    recipe[7].reagentName = "Xylene";
    recipe[8].reagentName = "Parafin Wax";
    recipe[8].time=30;
    recipe[8].volume=0;
  }
};

#endif
