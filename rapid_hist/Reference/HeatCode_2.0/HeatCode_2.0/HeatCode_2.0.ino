#include <Wire.h>

#define HEAT_PIN 10 // Positive PWM Power Supply in H Bridge
#define COOL_PIN 11 // Negtive PWM Power Supply in H Bridge
#define THERMISTOR_PIN A3 

#define FILTER_SIZE 15//Filter size of median filter


/*Median filter to reduce voltage jitter*/
class MedianFilter {
private:
    int data[FILTER_SIZE]; // array to save data
    int index;             // currunt inserting
public:
    MedianFilter() : index(0) {
        // initialize
        for (int i = 0; i < FILTER_SIZE; i++) {
            data[i] = 0;
        }
    }
    int filter(int samples[]) {
        //copy into an array
        int sortedData[FILTER_SIZE];
        for (int i = 0; i < FILTER_SIZE; i++) {
            sortedData[i] = samples[i];
        }

        // sorting
        for (int i = 0; i < FILTER_SIZE - 1; i++) {
            for (int j = i + 1; j < FILTER_SIZE; j++) {
                if (sortedData[i] > sortedData[j]) {
                    int temp = sortedData[i];
                    sortedData[i] = sortedData[j];
                    sortedData[j] = temp;
                }
            }
        }

        // return median
        return sortedData[FILTER_SIZE / 2];
    }
};

class PIDController {
private:
  // PID Parameters
  float Kp, Ki, Kd;
  //Could be set with Controller.setTarget(float)
  float setpoint;
  //memorize summing error
  float sum_error;
  float last_error;

public:
  // Constructor
  PIDController(float kp, float ki, float kd, float target)
    : Kp(kp), Ki(ki), Kd(kd), setpoint(target), sum_error(0), last_error(0) {}

  // Set of Target
  void setTarget(float target) {
    setpoint = target;
    sum_error = 0;last_error = 0;
  }

  // Compute PID
  int compute(float temperature) {
    //Calculate error
    float error = setpoint - temperature;
    //Amount of P
    float P_out = Kp * error;
    //Amount of I
    sum_error += error;
    float I_out = Ki * sum_error;
    //Amount of D
    float derivative = error - last_error;
    float D_out = Kd * derivative;
    //Sum-up and Update error
    float output =constrain(P_out + I_out + D_out, 0, 255);
    last_error = error;



    //Debugging Logic
    float dutyRatio = 100 * output / 255;
    Serial.print("Duty Ratio:");
    Serial.println(dutyRatio);

    return (int)output;
  }
};

float readTemperature(int sampleInterval = 1000) {
  /*
 *@brief             : Read voltage from Pin3, calculate temprature according to datasheet of thermistor
 *@int sampleInterval: Sample interval in ms, default 1s(1000ms)
 *@return temperature: Temperature in celcius, float
*/
  //Internal Var                                        
  const float knownResistor = 9840.0;
  const float nominalResistance = 100000.0;
  const float nominalTemp =  25.0 + 273.15;
  const float BCoefficient = 3950.0;
  const float VSupply =5;
  int adcVoltage; 
  int adcReadings[FILTER_SIZE];
  float readVolt,thermistorResistance,temperature;
  static MedianFilter medianFilter;
 for (int i = 0; i < FILTER_SIZE; i++) {
        adcReadings[i] = analogRead(THERMISTOR_PIN);
        delay(2);
    }
  
  adcVoltage = medianFilter.filter(adcReadings);
  readVolt =  (adcVoltage/1023.0)*VSupply;
  thermistorResistance = knownResistor * (readVolt / (VSupply - readVolt));
  //Calculate R_therm with Steinhart Equation
  temperature=1.0 / ((log(thermistorResistance / nominalResistance) / BCoefficient) + (1.0 / nominalTemp)) - 273.15;

  //Debugging Logic
  Serial.print("Raw Voltage:");
  Serial.println(adcVoltage);
  Serial.print("Thermistor Resistance:");
  Serial.println(thermistorResistance);
  Serial.print("Temperature:");
  Serial.println(temperature);

  //Delay
  delay(sampleInterval);
  return temperature;
}
void setup() {

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  Serial.begin(9600);

}
void loop() {
  float targetTemp = 42;
  float currentTemp = readTemperature(1000);//interval:0.1s
  float p(15),i(0.02),d(0.5);//Hyperparameters p,i,d
  PIDController HeatPID = PIDController(p,i,d,targetTemp);
  while (1) {
    currentTemp = readTemperature(1000);
    int dutyRatioInt=HeatPID.compute(currentTemp);
    analogWrite(HEAT_PIN,dutyRatioInt);
  }
}