/**
 * PCF8575引脚映射头文件
 * 
 * 功能：将PCF8575的0-15号引脚映射为Pico的30-45号虚拟引脚
 * 映射关系：PCF8575引脚0 → 虚拟引脚30，PCF8575引脚1 → 虚拟引脚31，以此类推
 * 
 * 使用方法：
 * 1. 在代码中直接使用虚拟引脚号（30-45）
 * 2. 调用PCF8575_init()初始化PCF8575
 * 3. 调用PCF8575_update()更新引脚状态
 */

#pragma once

#include <Arduino.h>
#include "PCF8575.h"

// PCF8575配置
#define PCF8575_ADDR 0x20
#define PCF8575_VIRTUAL_PIN_BASE 50  // 映射为GP50-GP65号引脚
#define PCF8575_PIN_COUNT 16         // 虚拟引脚数量（对应物理引脚0-7, 10-17）

// PCF8575对象
extern PCF8575 pcf8575;

// 虚拟引脚到物理引脚的映射表
// PCF8575实际物理引脚：0-7, 10-17（没有8和9）
// 虚拟引脚50-65映射到物理引脚0-7, 10-17
static const uint8_t VIRTUAL_TO_PCF8575_MAP[16] = {
    0,   // 虚拟50 -> 物理0
    1,   // 虚拟51 -> 物理1
    2,   // 虚拟52 -> 物理2
    3,   // 虚拟53 -> 物理3
    4,   // 虚拟54 -> 物理4
    5,   // 虚拟55 -> 物理5
    6,   // 虚拟56 -> 物理6
    7,   // 虚拟57 -> 物理7
    8,  // 虚拟58 -> 物理10（跳过8和9）
    9,  // 虚拟59 -> 物理11
    10,  // 虚拟60 -> 物理12
    11,  // 虚拟61 -> 物理13
    12,  // 虚拟62 -> 物理14
    13,  // 虚拟63 -> 物理15
    14,  // 虚拟64 -> 物理16
    15   // 虚拟65 -> 物理17
};

// 虚拟引脚范围检查
#define IS_PCF8575_VIRTUAL_PIN(pin) ((pin) >= PCF8575_VIRTUAL_PIN_BASE && (pin) < PCF8575_VIRTUAL_PIN_BASE + PCF8575_PIN_COUNT)

// 虚拟引脚转换为PCF8575实际引脚（使用映射表）
#define VIRTUAL_TO_PCF8575_PIN(pin) (VIRTUAL_TO_PCF8575_MAP[(pin) - PCF8575_VIRTUAL_PIN_BASE])

/**
 * @brief 初始化PCF8575
 * @return true: 初始化成功, false: 初始化失败
 */
bool PCF8575_init();

/**
 * @brief 更新所有PCF8575引脚状态
 * 需要在loop()中定期调用
 */
void PCF8575_update();

/**
 * @brief 设置PCF8575虚拟引脚模式（模拟pinMode）
 * @param pin 虚拟引脚号（30-45）
 * @param mode 模式（OUTPUT/INPUT）
 */
void PCF8575_pinMode(uint8_t pin, uint8_t mode);

/**
 * @brief 设置PCF8575虚拟引脚电平（模拟digitalWrite）
 * @param pin 虚拟引脚号（30-45）
 * @param value 电平（HIGH/LOW）
 */
void PCF8575_digitalWrite(uint8_t pin, uint8_t value);

/**
 * @brief 读取PCF8575虚拟引脚电平（模拟digitalRead）
 * @param pin 虚拟引脚号（30-45）
 * @return 引脚电平（HIGH/LOW）
 */
int PCF8575_digitalRead(uint8_t pin);

/**
 * @brief 设置PCF8575虚拟引脚PWM（模拟analogWrite）
 * @param pin 虚拟引脚号（30-45）
 * @param value PWM值（0-255）
 */
void PCF8575_analogWrite(uint8_t pin, uint8_t value);

// Arduino标准函数重写，用于透明替换
void pinMode_PCF8575(uint8_t pin, uint8_t mode);
void digitalWrite_PCF8575(uint8_t pin, uint8_t value);
int digitalRead_PCF8575(uint8_t pin);
void analogWrite_PCF8575(uint8_t pin, uint8_t value);