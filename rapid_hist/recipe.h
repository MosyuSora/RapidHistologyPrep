#ifndef RECIPE_H
#define RECIPE_H

#include <Arduino.h>
#include <string.h>
#include <math.h>



struct formStep {
  String reagentName;
  float volume;
  float deadVolume;
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
    recipe[1].reagentName = "70%Ethanol I";
    recipe[2].reagentName = "70%Ethanol II";
    recipe[3].reagentName = "95%Ethanol I";
    recipe[4].reagentName = "95%Ethanol II";
    recipe[5].reagentName = "Xylene I";
    recipe[6].reagentName = "Xylene II";
    recipe[7].reagentName = "Xylene III";
    recipe[8].reagentName = "Parafin Wax";
    recipe[8].time=30;
    recipe[8].volume=0;

    //Designed Dead Volume;

    recipe[0].deadVolume =8.89;
    recipe[1].deadVolume =8.52;
    recipe[2].deadVolume =8.52;
    recipe[3].deadVolume =8.38;
    recipe[4].deadVolume =8.38;
    recipe[5].deadVolume =8.84;
    recipe[6].deadVolume =8.84;
    recipe[7].deadVolume =8.84;

    



  }
};

#endif
