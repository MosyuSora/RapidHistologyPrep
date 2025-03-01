#include <Wire.h>

#define HEAT_PIN 10         // 加热器 PWM 控制（未测试加热部分）
#define COOL_PIN 11         // 制冷器 PWM 控制（未使用）
#define THERMISTOR_PIN A3   // 温度传感器模拟输入
#define BUTTON_PIN 2        // Pushbutton 接在 D2
#define LED_GREEN_PIN 12    
#define LED_RED_PIN 13      
#define LED_BLUE_PIN 8
#define FILTER_SIZE 15      // 中值滤波器采样数


/* 中值滤波器类，用于减少模拟信号抖动 */
class MedianFilter {
private:
    int data[FILTER_SIZE]; // 存储采样数据
    int index;             // 当前插入位置（本例中未循环使用）
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
        // 简单排序，取中位数
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

/* PID 控制器类 */
class PIDController {
private:
    float Kp, Ki, Kd;    // PID 参数
    float setpoint;      // 目标温度
    float sumError;     // 积分累计
    float lastError;    // 上一次误差（用于微分项）
    float dutyRatio;
public:
    PIDController(float kp, float ki, float kd, float target)
      : Kp(kp), Ki(ki), Kd(kd), setpoint(target), sumError(0), lastError(0) ,dutyRatio(0){}

    void setTarget(float target) {
      setpoint = target;
      sumError = 0;
      lastError = 0;
    }

    int compute(float temperature) {
      float error = setpoint - temperature;
      float P_out = Kp * error;
      float maxIntegral = 100;  // 设定一个合适的积分上限
      if (abs(error) < 1.0) {  // 误差足够小时，才允许积分作用
        sumError += error;
        sumError = constrain(sumError, -maxIntegral, maxIntegral);
      }   
      float I_out = Ki * sumError;
      float derivative = error - lastError;
      float D_out = Kd * derivative;
      float output = constrain(P_out + I_out + D_out, 0, 255);
      lastError = error;

      // 调试输出占空比百分比
      dutyRatio = 100 * output / 255;
      return (int)output;
    }
    void ratioSerialPrint(){
      Serial.print("Duty Ratio: ");
      Serial.println(dutyRatio);
    }
};

/*加热相关全局变量名称空间*/
namespace heatingPara{
  float targetTemp = 40.0; // 初始目标温度
  uint16_t tempSetpointCnt = 0;//用于温度达成判断逻辑
  float p = 15, i = 0.02, d = 0.5;
  PIDController HeatPID(p, i, d, targetTemp);//实例化类

}

/*按钮相关全局变量名称空间*/
namespace buttonPara{
  bool lastButtonState = HIGH;
}

/*led控制相关全局变量名称空间*/
namespace ledPara{
  unsigned long lastBlinkTime = 0;
  int blinkInterval = 1000; // 低温（40°C）慢闪
  bool ledState = LOW;
}

/*温度ADC采样函数*/
float readTemperature(int sampleInterval = 1000) {
  // 静态变量保存采样状态（保持多次调用间的状态）
  static enum { IDLE, SAMPLING, CALCULATING } state = IDLE;
  static unsigned long lastSampleTime = 0;
  static int adcIndex = 0;
  static int adcReadings[FILTER_SIZE];
  
  // 硬件参数
  const float knownResistor = 9840.0;
  const float nominalResistance = 100000.0;
  const float nominalTemp = 25.0 + 273.15;
  const float BCoefficient = 3950.0;
  const float VSupply = 5;

  switch(state) {
    case IDLE:
      if (millis() - lastSampleTime >= sampleInterval) {
        // 启动新采样周期
        state = SAMPLING;
        adcIndex = 0;
        memset(adcReadings, 0, sizeof(adcReadings));
      }
      break;

    case SAMPLING:
      if (adcIndex < FILTER_SIZE) {
        // 非阻塞采样
        static unsigned long lastADC = 0;
        if (millis() - lastADC >= 2) {
          adcReadings[adcIndex++] = analogRead(THERMISTOR_PIN);
          lastADC = millis();
        }
      } else {
        // 采样完成，进入计算
        state = CALCULATING;
      }
      break;

    case CALCULATING:
      // 中值滤波计算
      MedianFilter filter;
      int adcVal = filter.filter(adcReadings);
      
      // 温度计算
      float readVolt = (adcVal / 1023.0) * VSupply;
      float thermistorR = knownResistor * (readVolt / (VSupply - readVolt));
      float temp = 1.0 / ((log(thermistorR / nominalResistance) / BCoefficient) 
                          + (1.0 / nominalTemp)) - 273.15;

      // 调试输出
      Serial.print("Raw Voltage: ");
      Serial.println(adcVal);
      Serial.print("Thermistor Resistance: ");
      Serial.println(thermistorR);
      Serial.print("Temperature: ");
      Serial.println(temp);
      heatingPara::HeatPID.ratioSerialPrint();

      // 准备下次采样
      state = IDLE;
      lastSampleTime = millis();
      return temp;  // 返回有效温度值
  }
  
  return -999;  // 采样未完成时返回无效值（需调用方处理）
}
/*按钮处理函数*/
void handleButtonPress() {
    bool reading = digitalRead(BUTTON_PIN);
    if (reading == LOW && buttonPara::lastButtonState == HIGH) {
        // 切换目标温度
        heatingPara::targetTemp = (heatingPara::targetTemp == 40.0) ? 60.0 : 40.0;
        heatingPara::HeatPID.setTarget(heatingPara::targetTemp);
        // 根据目标温度红灯闪烁频率
        ledPara::blinkInterval = (heatingPara::targetTemp == 40.0) ? 1000 : 200; // 低温慢闪，高温快闪
        // 熄灭蓝灯，重置设定值检测
        digitalWrite(LED_BLUE_PIN, LOW);
        heatingPara::tempSetpointCnt=0;

        
    }
    buttonPara::lastButtonState = reading;
}

/*控制 LED 闪烁函数*/
void updateLEDStatus() {
  //红灯逻辑：根据给的blinkInterval闪烁
    unsigned long currentMillis = millis();
    unsigned long halfInterval = ledPara::blinkInterval / 2; 
    if (currentMillis - ledPara::lastBlinkTime >= halfInterval) {
        ledPara::lastBlinkTime = currentMillis;
        ledPara::ledState = !(ledPara::ledState);
        digitalWrite(LED_RED_PIN, ledPara::ledState);
    }
  //蓝灯逻辑:达到设定点后常量（由按钮处理函数关闭）
    if (heatingPara::tempSetpointCnt>=10){
        digitalWrite(LED_BLUE_PIN, HIGH);
    }
}
/*加热PWM输出+串口打印函数*/
void controlHeating() {
    float currentTemp = readTemperature(1000);
    int dutyRatioInt = heatingPara::HeatPID.compute(currentTemp);
    analogWrite(HEAT_PIN, dutyRatioInt);
    if(abs(currentTemp-heatingPara::targetTemp)<=1&&(heatingPara::tempSetpointCnt<128)){
      heatingPara::tempSetpointCnt++;//8-bit cnt,避免overflow
    }
}

/*初始化*/
void setup() {

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);  // 按钮使用外部上拉
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, HIGH);//电源指示灯（Green）常亮
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  //串口波特率
  Serial.begin(9600);
}

/*主程序*/
void loop() {
    controlHeating();
    handleButtonPress();
    updateLEDStatus();
    
}

