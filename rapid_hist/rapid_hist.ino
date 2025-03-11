#include <Wire.h>
#include <math.h>
// ========== Pin Definitions ==========
#define BOARD_BTN 2         // On-board button for user input or control trigger
#define STEP_PIN_1 4        // Step pin for Motor 1
#define DIR_PIN_1 5         // Direction pin for Motor 1
#define SLP_PIN_1 6         // Sleep pin for Motor 1
#define STEP_PIN_2 7        // Step pin for Motor 2
#define LED_BLUE_PIN 8      // Blue LED pin
#define HEAT_PIN 10         // Heater PWM control
#define COOL_PIN 11         // Cooler PWM control
#define LED_GREEN_PIN 12    // Green LED pin
#define LED_RED_PIN 13      // Red LED pin
#define DIR_PIN_2 15        // Direction pin for Motor 2 (Analog A1)
#define SLP_PIN_2 16        // Sleep pin for Motor 2 (Analog A2)
#define THERMISTOR_PIN A3   // Thermistor analog input pin
#define FILTER_SIZE 15      // Number of samples for median filtering

// ========== Classes ==========
/* Median Filter class to reduce signal noise */
  class MedianFilter {
  private:
      int data[FILTER_SIZE]; // Storage for sampling data
      int index;             // Current insertion index (unused in this version)

  public:
      MedianFilter() : index(0) {
          for (int i = 0; i < FILTER_SIZE; i++) {
              data[i] = 0;
          }
      }

      // Perform median filtering on the input samples
      int filter(int samples[]) {
          int sortedData[FILTER_SIZE];
          for (int i = 0; i < FILTER_SIZE; i++) {
              sortedData[i] = samples[i];
          }
          // Simple sorting algorithm to find the median
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
  };

  /* PID Controller class */
  class PIDController {
  private:
      float Kp, Ki, Kd;     // PID constants
      float setpoint;       // Target temperature 
      float sumError;       // Integral sum
      float lastError;      // Last error value (for derivative calculation)
      float dutyRatio;      // PWM duty ratio for debugging

  public:
      PIDController(float kp, float ki, float kd, float target)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(target), sumError(0), lastError(0) ,dutyRatio(0){}

      void setTarget(float target) {
        setpoint = target;
        sumError = 0;
        lastError = 0;
      }

      // Calculate PID output based on the current temperature
      int compute(float temperature) {
        float error = setpoint - temperature;
        float P_out = Kp * error;
        float maxIntegral = 150;  // Maximum integral limit
        if (abs(error) < 5.0) {   // Only accumulate integral for small errors
          sumError += error;
          sumError = constrain(sumError, -maxIntegral, maxIntegral);
        }   
        float I_out = Ki * sumError;
        float derivative = error - lastError;
        float D_out = Kd * derivative;
        float output = constrain(P_out + I_out + D_out, 0, 255);
        lastError = error;

        dutyRatio = 100 * output / 255;  // Calculate duty ratio for debugging
        return (int)output;
      }

      void ratioSerialPrint(){
        Serial.print("Duty Ratio: ");
        Serial.println(dutyRatio);
      }

  };

  /* Stepper Motor control class */
  class stepMotor {
  private:
        int stepPin, dirPin, slpPin;  // Pins for stepping, direction, and sleep control
        int stepDelay;                // Delay between steps in microseconds
        unsigned long lastStepTime;   // Timestamp of the last step taken
        int currentStep, totalSteps;  // Current and total steps for movement
        bool stepHigh;                // State of the step pin (HIGH or LOW)
        bool running;                 // Indicates if the motor is running

  public:
      // Constructor to initialize motor pins and delay
      stepMotor(int step, int dir, int slp, int delayMicros)
          : stepPin(step), dirPin(dir), slpPin(slp), stepDelay(delayMicros),
            lastStepTime(0), currentStep(0), totalSteps(0), stepHigh(false), running(false) {}

      // Start motor movement for a specific number of steps and direction
      // 'forward' controls the rotation direction (true for forward, false for reverse)
      void start(int steps, bool forward) {
          totalSteps = steps;
          currentStep = 0;
          running = true;

          digitalWrite(dirPin, forward ? HIGH : LOW); // Set rotation direction
          digitalWrite(slpPin, HIGH);                 // Wake up the motor
      }
          
      // Update motor position based on timing
      void update() {
          if (!running) return;                       // Exit if the motor is not running
          unsigned long now = micros();
          if (now - lastStepTime >= stepDelay) {      // Check if it's time for the next step
              lastStepTime = now;
              digitalWrite(stepPin, stepHigh ? HIGH : LOW);
              stepHigh = !stepHigh;

              if (!stepHigh) {                       // Increment step count on falling edge
                  currentStep++;
                  if (currentStep >= totalSteps) {   // Stop when target steps are reached
                      running = false;
                      digitalWrite(slpPin, LOW);     // Put the motor back to sleep
                  }
              }
          }
      }

      bool isIdle() { return !running; }
};

// ========== Namespace for Global Variables ==========
/* Namespace for heating-related global variables */
  namespace heatingPara{
    float targetTemp = 40.0;                      // Initial target temperature in degrees Celsius
    uint16_t tempSetpointCnt = 0;                 // Counter to determine if the target temperature is reached
    float p = 15, i = 0.02, d = 0.5;              // PID controller constants (Proportional, Integral, Derivative)
    PIDController HeatPID(p, i, d, targetTemp);   // Instantiate the PID controller with initial parameters
     // Enumeration for the different phases of the heating process
    enum class Phase { IDLE, HEAT_40, SOAK_40, HEAT_60, SOAK_60, COOL_40 } phase = Phase::IDLE;
    unsigned long phaseStartTime = 0; // Initialize to IDLE phase
  }

/* Namespace for button control global variables */
  namespace buttonPara{
    bool lastButtonState = HIGH;                  // Track the last state of the button (HIGH for unpressed)
  }

/* Namespace for LED control global variables */
  namespace ledPara{
    unsigned long lastBlinkTime = 0;              // Timestamp of the last LED state change
    int blinkInterval = 32767;                    // Initial blinking interval for LED (set to very slow)
    bool ledState = LOW;                          // Current state of the LED (LOW for off)
  }

 /* Namespace for fluid control global variables */
  namespace fluidPara {
    // Constants for fluid volume calculation
      const double pi = M_PI;
      const double ID = 0.4;                      // Tubing ID in cm
      const double TLengthin = 25.0;              // Tubing Length in cm for input
      const double TLengthout = 25.0;             // Tubing Length in cm for output
      const double flow = 3.0/5.0;                // Volumetric flow rate in mL/rotation

    // Fluid control state machine definition
    // System idle
    // Initial pump preparation
    // Pumping fluid into the system
    // Reversing flow for backwash
    // Soaking phase for fluid interaction
    // Pumping fluid out of the system
    // Waiting between cycles
      enum class FluidState {
          IDLE, INIT_PUMP, PUMP_IN, BACKFLOW, SOAKING, PUMP_OUT, WAIT_CYCLE
      } state = FluidState::IDLE;

      int cycleIndex = 0;                          // Index to track the current cycle
      int steps = 0;                               // Steps required for motor to move specific volume
      bool backflow = false;                       // Indicator if backflow is required in the current cycle
      unsigned long lastTime = 0;                  // Timestamp for tracking delays in processes
      unsigned long soakStartTime = 0;             // Timestamp when soaking starts

    // Soak times for each stage (modifiable via serial screen in the future)
    // const int soakTimes[8] = {10, 5, 5, 5, 5, 5, 5, 5}; // 每个阶段的浸泡时间（分钟）
      const float soakTimes[8] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; // Test soak times in minutes
    // Stepper motor objects for inlet and outlet fluid control
      stepMotor motorInlet(STEP_PIN_1, DIR_PIN_1, SLP_PIN_1, 1750);
      stepMotor motorOutlet(STEP_PIN_2, DIR_PIN_2, SLP_PIN_2, 1750);
  }

//========= Functions: Fluid Pumping and Time Management ===========
/* Calculate the number of steps required for a given fluid volume */
  int calculateSteps(float volume) {
      return ceil(volume / fluidPara::flow) * 800; // Convert volume to motor steps based on flow rate           
  }

/* Initiate the fluid pumping cycle */
  void startFluidCycle() {
      if (fluidPara::state == fluidPara::FluidState::IDLE) {
          fluidPara::state = fluidPara::FluidState::INIT_PUMP;   // Start with initial pump preparation
          fluidPara::cycleIndex = 0;                             // Reset cycle index
          fluidPara::lastTime = millis();                        // Record current time for timing control
      }
  }

/* Process the fluid pumping logic using a state machine */
  void processFluid() {
      using namespace fluidPara;
      unsigned long now = millis();

      switch (state) {
          case FluidState::IDLE:
              return;                                           // Do nothing when idle

          case FluidState::INIT_PUMP:
              // Determine steps based on cycle index
              steps = (cycleIndex == 2 || cycleIndex == 4 || cycleIndex == 7 || cycleIndex == 6)
                  ? calculateSteps(1)
                  : calculateSteps(3.14 * pow(ID / 2, 2) * TLengthin + 1);
              // Determine if backflow is required in the current cycle
              backflow = (cycleIndex == 0 || cycleIndex == 2 || cycleIndex == 4 || cycleIndex == 7);
              motorInlet.start(steps, true);                    // Start pumping fluid in
              state = FluidState::PUMP_IN;                      // Move to pumping-in state
              return;

          case FluidState::PUMP_IN:
              if (motorInlet.isIdle()) {                        // Wait until inlet pump is idle
                  if(backflow){
                    state = FluidState::BACKFLOW;
                    motorInlet.start(calculateSteps(1), false); // Initiate backflow by reversing direction
                  }else{
                    state = FluidState::SOAKING;                // Proceed to soaking phase
                    soakStartTime = millis();                   // Record soaking start time
                  }

              }
              return;

          case FluidState::BACKFLOW:
              if (motorInlet.isIdle()) {                        // Wait until backflow is complete
                state = FluidState::SOAKING;                    // Proceed to soaking phase
                soakStartTime = millis();
              } 
              return;

          case FluidState::SOAKING:
              // Check if soaking time has elapsed
              if (now - soakStartTime >= soakTimes[cycleIndex] * 60000) { //current soaking time（min)*60s
                  state = FluidState::PUMP_OUT;                 // Move to pumping out state
              }
              return;

          case FluidState::PUMP_OUT:
              // Calculate steps to pump out the fluid and initiate the outlet pump
              motorOutlet.start(calculateSteps(3.14 * pow(ID / 2, 2) * TLengthout + 1), true);
              state = FluidState::WAIT_CYCLE;                   // Transition to wait state
              lastTime = millis();                              // Record the start time of the wait
              return;

          case FluidState::WAIT_CYCLE:
              // Wait for 5 seconds to ensure the system is ready for the next cycle
              if (now - lastTime >= 5000) {                     //5s
                  cycleIndex++;
                  if (cycleIndex < 8) {
                      state = FluidState::INIT_PUMP;            // Continue to the next cycle
                  } else {                                      //After 8 cycles
                      state = FluidState::IDLE;                 // All cycles completed, go idle
                      heatingPara::phase = heatingPara::Phase::HEAT_60;// Transition to next heating phase
                      tempSetInit();                            // Initialize temperature settings for the next phase
                  }
              }
              return;
      }
  }
  
/* Real-time motor control function */
  void motorControl(){
    fluidPara::motorInlet.update();                             // Update the inlet motor state
    fluidPara::motorOutlet.update();                            // Update the outlet motor state
  }


//========= Functions: Heating Control and Temperature Sampling ===========//
/* Temperature ADC sampling function */
  float readTemperature(int sampleInterval = 1000) {
    // Static variables to maintain state across function calls
    static enum { IDLE, SAMPLING, CALCULATING } state = IDLE;   // State machine for sampling
    static unsigned long lastSampleTime = 0;                    // Timestamp of the last sample
    static int adcIndex = 0;                                    // Current index for ADC readings
    static int adcReadings[FILTER_SIZE];                        // Array to store ADC readings for filtering
    
    // Hardware-specific constants
    const float knownResistor = 9840.0;                         // Known resistor value in the voltage divider
    const float nominalResistance = 100000.0;                   // Nominal resistance of thermistor at 25°C
    const float nominalTemp = 25.0 + 273.15;                    // Nominal temperature in Kelvin
    const float BCoefficient = 3950.0;                          // Beta coefficient for thermistor calculations
    const float VSupply = 5;                                    // Supply voltage

    switch(state) {
      case IDLE:
        // Start new sampling cycle if the interval has elapsed
        if (millis() - lastSampleTime >= sampleInterval) {
          state = SAMPLING;
          adcIndex = 0;
          memset(adcReadings, 0, sizeof(adcReadings));          // Clear previous readings
        }
        break;

      case SAMPLING:
        if (adcIndex < FILTER_SIZE) {
          // Perform non-blocking sampling at a 2ms interval
          static unsigned long lastADC = 0;
          if (millis() - lastADC >= 2) {
            adcReadings[adcIndex++] = analogRead(THERMISTOR_PIN);// Read ADC value
            lastADC = millis();
          }
        } else {
          state = CALCULATING;                                  // Move to calculation phase once sampling is complete
        }
        break;

      case CALCULATING:
        // Apply median filtering to smooth out noise
        MedianFilter filter;
        int adcVal = filter.filter(adcReadings);
        
        // Calculate voltage and thermistor resistance
        float readVolt = (adcVal / 1023.0) * VSupply;
        float thermistorR = knownResistor * (readVolt / (VSupply - readVolt));
        // Calculate temperature using the Steinhart-Hart equation
        float temp = 1.0 / ((log(thermistorR / nominalResistance) / BCoefficient) 
                            + (1.0 / nominalTemp)) - 273.15;

        // Debug output to Serial Monitor
        Serial.print("Raw Voltage: ");
        Serial.println(adcVal);
        Serial.print("Thermistor Resistance: ");
        Serial.println(thermistorR);
        Serial.print("Temperature: ");
        Serial.println(temp);
        heatingPara::HeatPID.ratioSerialPrint();
        Serial.print("Heating SM number: ");
        Serial.println(static_cast<int>(heatingPara::phase));
        Serial.print("Fluid SM number: ");
        Serial.println(static_cast<int>(fluidPara::state));
        // Prepare for the next sampling cycle
        state = IDLE;
        lastSampleTime = millis();
        return temp;  // Return calculated temperature
    }
    
    return -999;  // Return an invalid value if sampling is incomplete
  }

/* Temperature Reset Function (Set Target and Update Indicators) */
  void tempSetInit(){
    // If in IDLE phase, initialize to heating phase at 40°C
    if(heatingPara::phase == heatingPara::Phase::IDLE ){
      heatingPara::phase = heatingPara::Phase::HEAT_40;
    }
    // Set target temperature based on the current heating phase
    switch(heatingPara::phase) {
            case heatingPara::Phase::HEAT_60:
                heatingPara::targetTemp = 60.0;
                break;
            case heatingPara::Phase::COOL_40:
                heatingPara::targetTemp = 40.0;
                break;
            default: break;  // No action needed for other phases
        }
    heatingPara::HeatPID.setTarget(heatingPara::targetTemp); // Update PID controller with the new target temperature
    heatingPara::tempSetpointCnt = 0;                        // Reset the setpoint counter
    // Adjust red LED blinking interval based on the temperature target
    // Slow blink for low temperature (40°C) and fast blink for high temperature (60°C)
    ledPara::blinkInterval = (heatingPara::targetTemp == 40.0) ? 1000 : 200;
    // Turn off the blue LED during setup
    digitalWrite(LED_BLUE_PIN, LOW);

  }



/* Heating Control Function with PWM and Serial Output */
  void heatingControl() {
      using namespace heatingPara;
      // Read the current temperature
      float currentTemp = readTemperature(1000);
      if (currentTemp == -999) return;                      // Exit if temperature reading is invalid
      // If the temperature is within ±0.4°C of the target and the counter is below 128, increment the counter
      if(abs(currentTemp-heatingPara::targetTemp)<=0.4&&(heatingPara::tempSetpointCnt<128)){
        heatingPara::tempSetpointCnt++;
      }
      // Compute the PWM output using the PID controller and apply it to the heating element
      int dutyRatioInt = heatingPara::HeatPID.compute(currentTemp);
      analogWrite(HEAT_PIN, dutyRatioInt);      

      // State machine to manage the heating process phases
      switch(phase) {
          case Phase::HEAT_40:
              if (tempSetpointCnt >= 10) {                  // Transition to soaking phase at 40°
                  phase = Phase::SOAK_40;
              }
              break;

          case Phase::SOAK_40:
              startFluidCycle()
              break;

          case Phase::HEAT_60:
              if (tempSetpointCnt >= 10) {                 
                  phase = Phase::SOAK_60;                  // Transition to soaking at 60°C
                  phaseStartTime = millis();               // Record the start time of the phase
                  tempSetpointCnt = 0;                     // Reset the setpoint counter
              }
              break;

          case Phase::SOAK_60:
              // If soaking time of 0.5 minutes has elapsed, start cooling phase
              if (millis() - phaseStartTime >= 0.5*60000) {
                  phase = Phase::COOL_40;
                  tempSetInit();
                  
              }
              break;

          case Phase::COOL_40:
              if (tempSetpointCnt >= 10) { 
                  phase = Phase::IDLE; // Transition to IDLE once cooling is stabilized at 40°C
                  //ledPara::blinkInterval=32767;
              }
              break;
      }



      
  }

  
//========= Functions: Button and LED Indicators ==========//
/* Button Handling Function */
  void handleButton() {
    // Skip logic if the current phase is not IDLE
    if (heatingPara::phase != heatingPara::Phase::IDLE) return;
    bool reading = digitalRead(BOARD_BTN);                // Read the current button state
    // If button press is detected (positive edge detection)
    if (reading == LOW && buttonPara::lastButtonState == HIGH) {
        tempSetInit();
    }
    buttonPara::lastButtonState = reading;                // Update the last button state
  }

/* LED Status Control Function */
  void updateLEDStatus() {
      unsigned long currentMillis = millis();
      unsigned long halfInterval = ledPara::blinkInterval / 2; 
      // Red LED Logic: Blink based on configured interval (slow for low temp, fast for high temp)
      if (currentMillis - ledPara::lastBlinkTime >= halfInterval) {
          ledPara::lastBlinkTime = currentMillis;
          ledPara::ledState = !(ledPara::ledState);
          digitalWrite(LED_RED_PIN, ledPara::ledState);
      }
      // Blue LED Logic: Turn on when temperature reaches the setpoint for ten consecutive counts
      if (heatingPara::tempSetpointCnt>=10){
          digitalWrite(LED_BLUE_PIN, HIGH);
      }
      // Green LED Logic: Indicates power status, stays constantly on (configured during setup)
  }



//========= Main Program ==========//
/* Initialization */
void setup() {
  // Initialize motor control pins
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(SLP_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SLP_PIN_2, OUTPUT);
  
  // Initialize heating and cooling control pins
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(BOARD_BTN, INPUT);          // puse button uses pull up resistor

  // Initialize LED indicator pins
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  // Set initial LED states
  digitalWrite(LED_GREEN_PIN, HIGH); // Green LED stays ON to indicate power status
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);

  // Begin serial communication at 9600 baud rate
  Serial.begin(9600);
}

/* Main Loop */
void loop() {
    handleButton();                  // Manage button presse
    processFluid();                  // Handle fluid control and state management
    heatingControl();                // Manage heating using PID control
    motorControl();                  // Update motor status and steps
    updateLEDStatus();               // Update LED indicators based on system states
    
}

