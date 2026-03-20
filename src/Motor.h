#include <Arduino.h>
#pragma once

// 默认 PWM 占空比（0-255），具体值在 Motor.cpp 中定义
extern const uint8_t DEFAULT_PWM_DUTY_CYCLE;

class Motor{
private:
    int pin1;
    int pin2;
    uint8_t pwmDutyCycle = 0;
    bool isStop = true;
    String state = "停止";
public:
    Motor(int pin1, int pin2);
    void setDutyCycle(uint8_t dutyCycle);
    uint8_t getDutyCycle() const;
    void forward();
    void backforward();
    void stop();
    bool getStopState();
    String getState();
};
