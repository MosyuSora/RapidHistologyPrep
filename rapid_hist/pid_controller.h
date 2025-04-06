#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class pidController {
  private:
    float Kp, Ki, Kd;
    float setpoint;
    float sumError;
    float lastError;
    float dutyRatio;

  public:
    pidController(float kp, float ki, float kd, float target);
    void setTarget(float target);
    int compute(float temperature);
    float getHeatPwr();
};

#endif
