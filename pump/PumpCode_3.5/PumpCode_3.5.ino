#include <iostream>
#include <cmath>  // Include cmath library for math functions and constants

// defines pins
#define STEP_PIN_1 3
#define DIR_PIN_1 4
#define SLP_PIN_1 5 
#define STEP_PIN_2 6
#define DIR_PIN_2 7
#define SLP_PIN_2 16//A2
#define MOTOR_BTN 15//A1
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(STEP_PIN_1,OUTPUT); 
  pinMode(DIR_PIN_1,OUTPUT);
  pinMode(SLP_PIN_1,OUTPUT);
  pinMode(STEP_PIN_2,OUTPUT); 
  pinMode(DIR_PIN_2,OUTPUT);
  pinMode(SLP_PIN_2,OUTPUT);
  pinMode(MOTOR_PIN, INPUT_PULLUP); // Set MOTOR_PIN pin as input with internal pull-up resistor

  Serial.begin(9600);  // Start serial communication at 9600 baud rate

}

int main() {
    double pi = M_PI;
    double ID = 0.4; // Tubing ID in cm
    double TLengthin = 25; // Tubing Length in cm for input
    double TLengthout = 25; // Tubing Length in cm for output
    double flow = 3/5; // Volumetric flow rate in mL/rotation
}

void loop() {
  int motorButtonPin = digitalRead(MOTOR_PIN);  // Read the state of the MOTOR_PIN (HIGH or LOW)

  digitalWrite(DIR_PIN_1,HIGH); // Enables the Inlet motor clockwise
  digitalWrite(DIR_PIN_2,LOW); // Enables the Outlet motor counterclockwise

  if (motorButtonPin == HIGH) {
    Serial.println("Pressed");  // MOTOR_PIN is pressed  
    
    for (int i = 0; i < 8;i++) {   // For each fluid wash, run the appropriate cycle

      if (i == 0) {     // Formalin soak - 10 minute soak with full volumes
        t = 10
        volumein = pi * pow(ID/2 ,2) * TLengthin + 1; // Dead volume in mL on input
        rotationsin = volumein / flow; // Relvolutions required to pump necessary volume in mL for input
        stepsin = ceil(rotationsin) * 800; // Number of microsteps needed to achieve the correct volume (rounded up) for input
        backflow = "True"

      } else if (i == 1 || i == 3 || i == 5){    // 70E1, 95E1, Xylene1 - 5 minute soak with full volumes
        t = 5
        volumein = pi * pow(ID/2 ,2) * TLengthin + 1; // Dead volume in mL on input
        rotationsin = volumein / flow; // Relvolutions required to pump necessary volume in mL for input
        stepsin = ceil(rotationsin) * 800; // Number of microsteps needed to achieve the correct volume (rounded up) for input
        backflow = "False"

      } else if (i == 2 || i == 4 || i == 7){    // 70E2, 95E2, Xylene3 - 5 minute soak with 1mL volumes
        t = 5
        steps = ceil(1/flow) * 800
        backflow = "True"
      
      } else if (i == 6){    // Xylene2 - 5 minute soak with 1mL volumes 
        t = 5
        steps = ceil(1/flow) * 800
        backflow = "False"
      }

     
     
      // Fluid Inlet
      digitalWrite(SLP_PIN_1, HIGH); // Set the sleep pin to HIGH to wake up the driver

      for(int x = 0; x < steps; x++) {
        digitalWrite(STEP_PIN_1,HIGH); 
        delayMicroseconds(1750);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(STEP_PIN_1,LOW); 
        delayMicroseconds(1750); 
      }
      digitalWrite(SLP_PIN_1, LOW); // Set the sleep pin to LOW to set the driver to sleep


      // Pump fluid in tubing back to reservoirs when necessary
      if (backflow == "True") {
        digitalWrite(SLP_PIN_1, HIGH); // Set the sleep pin to HIGH to wake up the driver
        digitalWrite(DIR_PIN_1,LOW); // Enables the Inlet motor counter-clockwise

        for(int x = 0; x < steps; x++) {
          digitalWrite(STEP_PIN_1,HIGH); 
          delayMicroseconds(1750);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(STEP_PIN_1,LOW); 
          delayMicroseconds(1750); 
        }
        digitalWrite(DIR_PIN_1,HIGH); // Enables the Inlet motor clockwise
        digitalWrite(SLP_PIN_1, LOW); // Set the sleep pin to LOW to set the driver to sleep
      }


      // Set soak times (minutes * 60sec/min * 1000ms/sec)
      delay(t * 60000); 



      // Fluid Drainage
      digitalWrite(SLP_PIN_2, HIGH); // Set the sleep pin to HIGH to wake up the driver  

      volumeout = pi * pow(ID/2 ,2) * TLengthout + 1; // Dead volume in mL on output 
      rotationsout = volumeout / flow; // Relvolutions required to pump necessary volume in mL for output 
      stepsout = ceil(rotationsout) * 800; // Number of microsteps needed to achieve the correct volume (rounded up) for output

      for(int x = 0; x < stepsout; x++) {
        digitalWrite(STEP_PIN_2,HIGH); 
        delayMicroseconds(1750);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(STEP_PIN_2,LOW); 
        delayMicroseconds(1750); 
      }
      digitalWrite(SLP_PIN_2, LOW); // Set the sleep pin to LOW to set the driver to sleep

      // 5 second delay between pump out and pump in
      delay (5000);
    }

    // Until the MOTOR_PIN is not pressed, put the drivers to sleep
  } else {
    Serial.println("Not Pressed");  // MOTOR_PIN is not pressed
    digitalWrite(SLP_PIN_1, LOW); // Set the sleep pin to LOW to set the driver to sleep
    digitalWrite(SLP_PIN_2, LOW); // Set the sleep pin to LOW to set the driver to sleep
      
    }
  
}