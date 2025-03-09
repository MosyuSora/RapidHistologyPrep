#include <Wire.h>
#include <math.h>

// Pin definitions
#define BOARD_BTN 2        // On-board button
#define MOTOR_BTN 3        // Motor control board button
#define STEP_PIN_1 4       // Step pin for Motor 1
#define DIR_PIN_1 5        // Direction pin for Motor 1
#define SLP_PIN_1 6        // Sleep pin for Motor 1 
#define STEP_PIN_2 7       // Step pin for Motor 2
#define LED_BLUE_PIN 8     // Blue LED
#define HEAT_PIN 10        // PWM control for heater
#define COOL_PIN 11        // PWM control for cooler
#define LED_GREEN_PIN 12   // Green LED
#define LED_RED_PIN 13     // Red LED
#define DIR_PIN_2 15       // Direction pin for Motor 2 (A1)
#define SLP_PIN_2 16       // Sleep pin for Motor 2 (A2)
#define THERMISTOR_PIN A3  // Analog input for the temperature sensor
  
#define FILTER_SIZE 15     // Number of samples for the median filter

//================== Classes ==================//

/* MedianFilter class: reduces noise in analog signals using a median filter */
class MedianFilter {
private:
    int data[FILTER_SIZE];  // Array to store sample data
    int index;              // Current insertion index (not used cyclically in this example)
public:
    MedianFilter() : index(0) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            data[i] = 0;
        }
    }
    int filter(int samples[]) {
        int sortedData[FILTER_SIZE];
        for (int i = 0; i < FILTER_SIZE; i++) {
            sortedData[i] = samples[i];
        }
        // Simple bubble sort to find the median value
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

/* PIDController class: implements PID control for temperature regulation */
class PIDController {
private:
    float Kp, Ki, Kd;      // PID coefficients
    float setpoint;        // Target temperature
    float sumError;        // Accumulated error (integral term)
    float lastError;       // Previous error (for derivative term)
    float dutyRatio;       // Computed duty cycle percentage for debugging
public:
    PIDController(float kp, float ki, float kd, float target)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(target), sumError(0), lastError(0), dutyRatio(0) {}

    void setTarget(float target) {
        setpoint = target;
        sumError = 0;
        lastError = 0;
    }

    int compute(float temperature) {
        float error = setpoint - temperature;
        float P_out = Kp * error;
        float maxIntegral = 100;  // Limit for the integral term
        if (abs(error) < 1.0) {     // Only accumulate error if it is sufficiently small
            sumError += error;
            sumError = constrain(sumError, -maxIntegral, maxIntegral);
        }
        float I_out = Ki * sumError;
        float derivative = error - lastError;
        float D_out = Kd * derivative;
        float output = constrain(P_out + I_out + D_out, 0, 255);
        lastError = error;

        // Debug: print the duty cycle percentage
        dutyRatio = 100 * output / 255;
        return (int)output;
    }
    void ratioSerialPrint() {
        Serial.print("Duty Ratio: ");
        Serial.println(dutyRatio);
    }
};

/* Motor class: controls a stepper motor */
class Motor {
private:
    int stepPin, dirPin, slpPin;
    int stepDelay;              // Delay between steps in microseconds
    unsigned long lastStepTime; // Timestamp of the last step
    int currentStep, totalSteps;
    bool stepHigh;              // Current state of the step signal
    bool running;               // Indicates whether the motor is active
public:
    Motor(int step, int dir, int slp, int delayMicros)
        : stepPin(step), dirPin(dir), slpPin(slp), stepDelay(delayMicros),
          lastStepTime(0), currentStep(0), totalSteps(0), stepHigh(false), running(false) {}

    void start(int steps, bool forward) {
        totalSteps = steps;
        currentStep = 0;
        running = true;
        digitalWrite(dirPin, forward ? HIGH : LOW);
        digitalWrite(slpPin, HIGH);
    }

    void update() {
        if (!running) return;

        unsigned long now = micros();
        if (now - lastStepTime >= stepDelay) {
            lastStepTime = now;
            digitalWrite(stepPin, stepHigh ? HIGH : LOW);
            stepHigh = !stepHigh;
            if (!stepHigh) {  // Count a step on the falling edge
                currentStep++;
                if (currentStep >= totalSteps) {
                    running = false;
                    digitalWrite(slpPin, LOW);
                }
            }
        }
    }

    bool isIdle() { return !running; }
};

//================== Global Variables (Namespaces) ==================//

/* Namespace for heating parameters */
namespace heatingPara {
    float targetTemp = 40.0; // Initial target temperature
    uint16_t tempSetpointCnt = 0; // Counter for setpoint detection logic
    float p = 15, i = 0.02, d = 0.5;
    PIDController HeatPID(p, i, d, targetTemp); // PID controller instance for the heater
}

/* Namespace for button parameters */
namespace buttonPara {
    bool lastButtonState = HIGH;
}

/* Namespace for LED control parameters */
namespace ledPara {
    unsigned long lastBlinkTime = 0;
    int blinkInterval = 1000; // Blink interval in milliseconds (slow blink for 40°C)
    bool ledState = LOW;
}

/* Namespace for fluid control parameters */
namespace fluidPara {
    // Constants for flow calculation
    const double pi = M_PI;
    const double ID = 0.4;           // Inner diameter of the tubing (cm)
    const double TLengthin = 25.0;   // Input tubing length (cm)
    const double TLengthout = 25.0;  // Output tubing length (cm)
    const double flow = 3.0 / 5.0;   // Volumetric flow rate (mL per rotation)

    // Fluid control state machine
    enum class FluidState {
        IDLE, INIT_PUMP, PUMP_IN, BACKFLOW, SOAKING, PUMP_OUT, WAIT_CYCLE
    } state = FluidState::IDLE;

    int cycleIndex = 0;
    int steps = 0;
    bool backflow = false;
    unsigned long lastTime = 0;
    unsigned long soakStartTime = 0;
    // Soak durations for each stage (in minutes); can be dynamically set via a serial interface in the future
    const int soakTimes[8] = {10, 5, 5, 5, 5, 5, 5, 5};

    // Motor instances for fluid control
    Motor motorInlet(STEP_PIN_1, DIR_PIN_1, SLP_PIN_1, 1750);
    Motor motorOutlet(STEP_PIN_2, DIR_PIN_2, SLP_PIN_2, 1750);
}

//================== Functions: Fluid Pumping and Timing ==================//

/* Calculate the number of steps needed for a given volume */
int calculateSteps(float volume) {
    return ceil(volume / fluidPara::flow) * 800;
}

/* Start a new fluid cycle */
void startFluidCycle() {
    if (fluidPara::state == fluidPara::FluidState::IDLE) {
        fluidPara::state = fluidPara::FluidState::INIT_PUMP;
        fluidPara::cycleIndex = 0;
        fluidPara::lastTime = millis();
    }
}

/* Process the fluid pumping state machine */
void processFluid() {
    using namespace fluidPara;
    unsigned long now = millis();

    switch (state) {
        case FluidState::IDLE:
            return;

        case FluidState::INIT_PUMP:
            steps = (cycleIndex == 2 || cycleIndex == 4 || cycleIndex == 7 || cycleIndex == 6)
                ? calculateSteps(1)
                : calculateSteps(3.14 * pow(ID / 2, 2) * TLengthin + 1);
            backflow = (cycleIndex == 0 || cycleIndex == 2 || cycleIndex == 4 || cycleIndex == 7);
            motorInlet.start(steps, true);
            state = FluidState::PUMP_IN;
            return;

        case FluidState::PUMP_IN:
            if (motorInlet.isIdle()) {
                state = backflow ? FluidState::BACKFLOW : FluidState::SOAKING;
                soakStartTime = millis();
            }
            return;

        case FluidState::BACKFLOW:
            motorInlet.start(steps, false);
            state = FluidState::SOAKING;
            soakStartTime = millis();
            return;

        case FluidState::SOAKING:
            if (now - soakStartTime >= soakTimes[cycleIndex] * 60000) {
                state = FluidState::PUMP_OUT;
            }
            return;

        case FluidState::PUMP_OUT:
            motorOutlet.start(calculateSteps(3.14 * pow(ID / 2, 2) * TLengthout + 1), true);
            state = FluidState::WAIT_CYCLE;
            lastTime = millis();
            return;

        case FluidState::WAIT_CYCLE:
            if (now - lastTime >= 5000) {
                cycleIndex++;
                if (cycleIndex < 8) {
                    state = FluidState::INIT_PUMP;
                } else {
                    state = FluidState::IDLE;
                }
            }
            return;
    }
}

/* Handle the motor control board button press */
void handleMotorButton() {
    bool reading = digitalRead(MOTOR_BTN);
    if (reading == LOW && buttonPara::lastButtonState == HIGH) {
        startFluidCycle();
    }
    buttonPara::lastButtonState = reading;
}

//================== Functions: Heating Control ==================//

/* Read temperature using ADC and apply median filtering */
float readTemperature(int sampleInterval = 1000) {
    // Static variables preserve state across function calls
    static enum { IDLE, SAMPLING, CALCULATING } state = IDLE;
    static unsigned long lastSampleTime = 0;
    static int adcIndex = 0;
    static int adcReadings[FILTER_SIZE];
    
    // Hardware parameters for the thermistor circuit
    const float knownResistor = 9840.0;
    const float nominalResistance = 100000.0;
    const float nominalTemp = 25.0 + 273.15;
    const float BCoefficient = 3950.0;
    const float VSupply = 5;

    switch (state) {
        case IDLE:
            if (millis() - lastSampleTime >= sampleInterval) {
                // Start a new sampling cycle
                state = SAMPLING;
                adcIndex = 0;
                memset(adcReadings, 0, sizeof(adcReadings));
            }
            break;

        case SAMPLING:
            if (adcIndex < FILTER_SIZE) {
                // Non-blocking ADC sampling
                static unsigned long lastADC = 0;
                if (millis() - lastADC >= 2) {
                    adcReadings[adcIndex++] = analogRead(THERMISTOR_PIN);
                    lastADC = millis();
                }
            } else {
                // All samples acquired; proceed to calculation
                state = CALCULATING;
            }
            break;

        case CALCULATING:
            // Apply median filter to the collected samples
            MedianFilter filter;
            int adcVal = filter.filter(adcReadings);
            
            // Compute the temperature
            float readVolt = (adcVal / 1023.0) * VSupply;
            float thermistorR = knownResistor * (readVolt / (VSupply - readVolt));
            float temp = 1.0 / ((log(thermistorR / nominalResistance) / BCoefficient)
                                + (1.0 / nominalTemp)) - 273.15;

            // Debug output
            Serial.print("Raw Voltage: ");
            Serial.println(adcVal);
            Serial.print("Thermistor Resistance: ");
            Serial.println(thermistorR);
            Serial.print("Temperature: ");
            Serial.println(temp);
            heatingPara::HeatPID.ratioSerialPrint();

            // Reset state for the next sampling cycle
            state = IDLE;
            lastSampleTime = millis();
            return temp;
    }
    
    return -999;  // Return an error value if sampling is incomplete
}

/* Control heater output using PWM and output debug information */
void controlHeating() {
    float currentTemp = readTemperature(1000);
    int dutyRatioInt = heatingPara::HeatPID.compute(currentTemp);
    analogWrite(HEAT_PIN, dutyRatioInt);
    if (abs(currentTemp - heatingPara::targetTemp) <= 1 && (heatingPara::tempSetpointCnt < 128)) {
        heatingPara::tempSetpointCnt++; // Prevent overflow of the 8-bit counter
    }
}

//================== Functions: Button and LED Control ==================//

/* Handle on-board button events */
void handleBoardButton() {
    bool reading = digitalRead(BOARD_BTN);
    if (reading == LOW && buttonPara::lastButtonState == HIGH) {
        // Toggle the target temperature between 40°C and 60°C
        heatingPara::targetTemp = (heatingPara::targetTemp == 40.0) ? 60.0 : 40.0;
        heatingPara::HeatPID.setTarget(heatingPara::targetTemp);
        // Set red LED blink rate: slow for low temperature, fast for high temperature
        ledPara::blinkInterval = (heatingPara::targetTemp == 40.0) ? 1000 : 200;
        // Turn off blue LED and reset the setpoint counter
        digitalWrite(LED_BLUE_PIN, LOW);
        heatingPara::tempSetpointCnt = 0;
    }
    buttonPara::lastButtonState = reading;
}

/* Update LED status based on current conditions */
void updateLEDStatus() {
    // Red LED blinks based on the configured interval
    unsigned long currentMillis = millis();
    unsigned long halfInterval = ledPara::blinkInterval / 2;
    if (currentMillis - ledPara::lastBlinkTime >= halfInterval) {
        ledPara::lastBlinkTime = currentMillis;
        ledPara::ledState = !ledPara::ledState;
        digitalWrite(LED_RED_PIN, ledPara::ledState);
    }
    // Blue LED stays on once the temperature setpoint is reached (blue LED is turned off by the button handler)
    if (heatingPara::tempSetpointCnt >= 10) {
        digitalWrite(LED_BLUE_PIN, HIGH);
    }
}

//================== Main Program ==================//

/* Setup function: initializes pins and serial communication */
void setup() {
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1, OUTPUT);
    pinMode(SLP_PIN_1, OUTPUT);
    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);
    pinMode(SLP_PIN_2, OUTPUT);
    pinMode(MOTOR_BTN, INPUT_PULLUP); // Use internal pull-up resistor for motor button
    
    pinMode(HEAT_PIN, OUTPUT);
    pinMode(COOL_PIN, OUTPUT);
    pinMode(THERMISTOR_PIN, INPUT);
    pinMode(BOARD_BTN, INPUT);  // Use external pull-up resistor for board button
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    digitalWrite(LED_GREEN_PIN, HIGH); // Green LED indicates power (always on)
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);
}

/* Main loop: continuously update heating, button, fluid, and LED functions */
void loop() {
    controlHeating();
    handleBoardButton();
    handleMotorButton();
    processFluid();
    fluidPara::motorInlet.update();
    fluidPara::motorOutlet.update();
    updateLEDStatus();
}

