#include <Wire.h>
#include <math.h>
#define BOARD_BTN 2        // 板载按钮
#define STEP_PIN_1 4
#define DIR_PIN_1 5
#define SLP_PIN_1 6 
#define STEP_PIN_2 7
#define LED_BLUE_PIN 8
#define HEAT_PIN 10         // 加热器 PWM 控制
#define COOL_PIN 11         // 制冷器 PWM 控制
#define LED_GREEN_PIN 12    
#define LED_RED_PIN 13    
#define DIR_PIN_2 15//A1
#define SLP_PIN_2 16//A2
#define THERMISTOR_PIN A3   // 温度传感器模拟输入
  
#define FILTER_SIZE 15      // 中值滤波器采样数

//=========对象：类===========//
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
  /*步进电机类*/
  class stepMotor {
  private:
      int stepPin, dirPin, slpPin;
      int stepDelay;
      unsigned long lastStepTime;
      int currentStep, totalSteps;
      bool stepHigh;
      bool running;

  public:
      stepMotor(int step, int dir, int slp, int delayMicros)
          : stepPin(step), dirPin(dir), slpPin(slp), stepDelay(delayMicros),
            lastStepTime(0), currentStep(0), totalSteps(0), stepHigh(false), running(false) {}

      void start(int steps, bool forward) {//forward控制正反转（true 为正，false为反)
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

              if (!stepHigh) {  // 下降沿计步
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

//=========对象：名称空间（全局变量）===========//
  /*加热相关全局变量名称空间*/
  namespace heatingPara{
    float targetTemp = 40.0; // 初始目标温度
    uint16_t tempSetpointCnt = 0;//用于温度达成判断逻辑
    float p = 15, i = 0.02, d = 0.5;
    PIDController HeatPID(p, i, d, targetTemp);//实例化类

    enum class Phase { IDLE, HEAT_40, SOAK_40, HEAT_60, SOAK_60, COOL_40 } phase = Phase::IDLE;
    unsigned long phaseStartTime = 0; // 阶段计时起点
  }

  /*按钮相关全局变量名称空间*/
  namespace buttonPara{
    bool lastButtonState = HIGH;
  }

  /*led控制相关全局变量名称空间*/
  namespace ledPara{
    unsigned long lastBlinkTime = 0;
    int blinkInterval = 32767; // 初始极慢闪
    bool ledState = LOW;
  }
  /*流量控制全局变量名称空间 */
  namespace fluidPara {
    //流量计算常数
      const double pi = M_PI;
      const double ID = 0.4; // Tubing ID in cm
      const double TLengthin = 25.0; // Tubing Length in cm for input
      const double TLengthout = 25.0; // Tubing Length in cm for output
      const double flow = 3.0/5.0; // Volumetric flow rate in mL/rotation
    //流体控制状态机
      enum class FluidState {
          IDLE, INIT_PUMP, PUMP_IN, BACKFLOW, SOAKING, PUMP_OUT, WAIT_CYCLE
      } state = FluidState::IDLE;

      int cycleIndex = 0;
      int steps = 0;
      bool backflow = false;
      unsigned long lastTime = 0;
      unsigned long soakStartTime = 0;
    //浸泡时间，未来根据串口屏数据动态设置
      const int soakTimes[8] = {10, 5, 5, 5, 5, 5, 5, 5}; // 每个阶段的浸泡时间（分钟）

    //两个电机
      stepMotor motorInlet(STEP_PIN_1, DIR_PIN_1, SLP_PIN_1, 1750);
      stepMotor motorOutlet(STEP_PIN_2, DIR_PIN_2, SLP_PIN_2, 1750);
  }

//=========函数：试剂泵送和时间管理===========//
  /* 计算步数 */
  int calculateSteps(float volume) {
      return ceil(volume / fluidPara::flow) * 800;
  }

  /* 启动液体循环*/
  void startFluidCycle() {
      if (fluidPara::state == fluidPara::FluidState::IDLE) {
          fluidPara::state = fluidPara::FluidState::INIT_PUMP;
          fluidPara::cycleIndex = 0;
          fluidPara::lastTime = millis();
      }
  }

  /* 处理液体泵送逻辑（状态机）*/
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
              if (motorInlet.isIdle()) {// 确保电机完成泵入
                  if(backflow){
                    FluidState::state = FluidState::BACKFLOW;
                    motorInlet.start(calculateSteps(1), false);
                  }else{
                    FluidState::state = FluidState::SOAKING;//浸泡
                    soakStartTime = millis();//记录浸泡开始时间
                  }

              }
              return;

          case FluidState::BACKFLOW:
              if (motorInlet.isIdle()) { // 确保电机完成回流
                state = FluidState::SOAKING;//浸泡
                soakStartTime = millis();//记录浸泡开始时间
              } 
              return;

          case FluidState::SOAKING:
              if (now - soakStartTime >= soakTimes[cycleIndex] * 60000) {//当前配方浸泡时间（min)*60s
                  state = FluidState::PUMP_OUT;
              }
              return;

          case FluidState::PUMP_OUT:
              motorOutlet.start(calculateSteps(3.14 * pow(ID / 2, 2) * TLengthout + 1), true);
              state = FluidState::WAIT_CYCLE;
              lastTime = millis();
              return;

          case FluidState::WAIT_CYCLE:
              if (now - lastTime >= 5000) {//5s抽干
                  cycleIndex++;
                  if (cycleIndex < 8) {
                      state = FluidState::INIT_PUMP;//还在液体阶段内
                  } else {//完成8轮试剂后
                      state = FluidState::IDLE;//回到空闲
                      heatingPara::phase = heatingPara::Phase::HEAT_60;//温度进入石蜡状态
                      tempSetInit();
                  }
              }
              return;
      }
  }
  
  /*实时控制电机*/
  void motorControl(){
    fluidPara::motorInlet.update();
    fluidPara::motorOutlet.update();
  }
//=========函数：加热控制全局变量名称空间===========//
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
  /*温度重置函数（设定值 显示灯）*/
  void tempSetInit(){
    if(heatingPara::phase == heatingPara::Phase::IDLE ){
      heatingPara::phase = heatingPara::Phase::HEAT_40;
    }
    switch(heatingPara::phase) {
            case heatingPara::Phase::HEAT_60:
                heatingPara::targetTemp = 60.0;
                break;
            case heatingPara::Phase::COOL_40:
                heatingPara::targetTemp = 40.0;
                break;
            default: break;
        }
    heatingPara::HeatPID.setTarget(targetTemp);
    heatingPara::tempSetpointCnt = 0; // 重置计数
    ledPara::blinkInterval = (heatingPara::targetTemp == 40.0) ? 1000 : 200; // 改变红灯：低温慢闪，高温快闪
    digitalWrite(LED_BLUE_PIN, LOW);//熄灭蓝灯

  }



  /*加热PWM输出+串口打印函数*/
  void heatingControl() {
      using namespace heatingPara;
    //检查温度
      float currentTemp = readTemperature(1000);
      if (currentTemp == -999) return;
      if(abs(currentTemp-heatingPara::targetTemp)<=0.2&&(heatingPara::tempSetpointCnt<128)){
        heatingPara::tempSetpointCnt++;//误差为0.2度时，视为已到达设定点，统计到达次数
      }
    //控制PWM
      int dutyRatioInt = heatingPara::HeatPID.compute(currentTemp);
      analogWrite(HEAT_PIN, dutyRatioInt);      

    //状态机管理
      switch(phase) {
          case Phase::HEAT_40:
              if (tempSetpointCnt >= 10) { // 稳定在40℃
                  phase = Phase::SOAK_40;
              }
              break;

          case Phase::SOAK_40:
              startFluidCycle();//交给processFluid决定是否进入下一个状态
              break;

          case Phase::HEAT_60:
              if (tempSetpointCnt >= 10) { // 稳定在60℃
                  phase = Phase::SOAK_60;
                  phaseStartTime = millis();
                  tempSetpointCnt = 0;
              }
              break;

          case Phase::SOAK_60:
              if (millis() - phaseStartTime >= 20*60000) { // 20分钟
                  phase = Phase::COOL_40;
                  tempSetInit();
                  
              }
              break;

          case Phase::COOL_40:
              if (tempSetpointCnt >= 10) { 
                  phase = Phase::IDLE;// 稳定在40℃
                  //ledPara::blinkInterval=32767;
              }
              break;
      }



      
  }

  
//=========函数：按钮与指示灯===========//
  /*按钮处理函数*/
  void handleButton() {
    if (heatingPara::phase != heatingPara::Phase::IDLE) return;//非空闲状态直接跳过逻辑
    bool reading = digitalRead(BOARD_BTN);
    if (reading == LOW && buttonPara::lastButtonState == HIGH) {//posEdge
        tempSetInit();
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
    //蓝灯逻辑:达到设定点后常量（由handleButton()函数关闭）
      if (heatingPara::tempSetpointCnt>=10){//在heatingControl中，连续计数十次，视为到达设定点，蓝灯亮
          digitalWrite(LED_BLUE_PIN, HIGH);
      }
    //绿灯逻辑：是电源指示灯 保持常亮 在setup阶段已被设置（后续可以搭建电流采样电路增加报错逻辑）
  }



//=========主程序===========//
/*初始化*/
void setup() {
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(SLP_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SLP_PIN_2, OUTPUT);
  
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(BOARD_BTN, INPUT);  // 按钮使用外部上拉
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
    handleButton();
    processFluid();
    heatingControl();
    motorControl();
    updateLEDStatus();
    
}

