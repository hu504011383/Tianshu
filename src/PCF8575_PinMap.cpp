/**
 * PCF8575引脚映射实现文件
 * 
 * 实现PCF8575虚拟引脚功能，将PCF8575的0-15号引脚映射为Pico的30-45号虚拟引脚
 */

#include "PCF8575_PinMap.h"

// 使用在AMCU.cpp中已经定义的PCF8575对象
extern PCF8575 pcf8575;

// 引脚状态缓存
uint16_t pcf8575_pin_states = 0; // 当前引脚状态（16位，对应物理引脚0-15，但注意实际引脚是0-7,10-17）
uint16_t pcf8575_pin_modes = 0;  // 引脚模式（0=INPUT, 1=OUTPUT）
uint8_t pcf8575_pwm_values[18] = {0}; // PWM值缓存（覆盖物理引脚0-17）
bool pcf8575_initialized = false; // 初始化状态

bool PCF8575_init() {
    if (pcf8575_initialized) {
        return true;
    }
    
    if (!pcf8575.begin()) {
        pcf8575_initialized = false;
        return false;
    }
    
    // 初始化所有引脚为低电平
    pcf8575.write16(0);
    pcf8575_pin_states = 0;
    pcf8575_pin_modes = 0; // 默认所有引脚为INPUT
    
    pcf8575_initialized = true;
    return true;
}

void PCF8575_update() {
    if (!pcf8575_initialized) {
        return;
    }
    
    // 更新输出引脚状态
    pcf8575.write16(pcf8575_pin_states);
    
    // 读取输入引脚状态（如果需要的话）
    // 注意：PCF8575是输出扩展器，输入功能有限
    // uint16_t input_states = pcf8575.read16();
}

void PCF8575_pinMode(uint8_t pin, uint8_t mode) {
    if (!IS_PCF8575_VIRTUAL_PIN(pin)) {
        return;
    }
    
    uint8_t pcf_pin = VIRTUAL_TO_PCF8575_PIN(pin);
    
    if (mode == OUTPUT) {
        pcf8575_pin_modes |= (1 << pcf_pin);
    } else {
        pcf8575_pin_modes &= ~(1 << pcf_pin);
    }
}

void PCF8575_digitalWrite(uint8_t pin, uint8_t value) {
    if (!IS_PCF8575_VIRTUAL_PIN(pin)) {
        return;
    }
    
    uint8_t pcf_pin = VIRTUAL_TO_PCF8575_PIN(pin);
    
    if (value == HIGH) {
        pcf8575_pin_states |= (1 << pcf_pin);
    } else {
        pcf8575_pin_states &= ~(1 << pcf_pin);
    }
    
    // 立即更新PCF8575状态
    if (pcf8575_initialized) {
        pcf8575.write16(pcf8575_pin_states);
    }
}

int PCF8575_digitalRead(uint8_t pin) {
    if (!IS_PCF8575_VIRTUAL_PIN(pin)) {
        return LOW;
    }
    
    // PCF8575输入读取：需要先设置为高电平（上拉），然后读取实际输入状态
    uint8_t pcf_pin = VIRTUAL_TO_PCF8575_PIN(pin);
    
    // 检查引脚模式
    bool is_output = (pcf8575_pin_modes & (1 << pcf_pin));
    
    if (is_output) {
        // 输出模式：返回当前设置的输出状态
        return (pcf8575_pin_states & (1 << pcf_pin)) ? HIGH : LOW;
    } else {
        // 输入模式：读取实际输入状态
        // PCF8575作为输入时需要先写入高电平（上拉）
        uint16_t current_states = pcf8575_pin_states;
        current_states |= (1 << pcf_pin);  // 设置对应位为高电平
        pcf8575.write16(current_states);   // 写入上拉
        
        // 读取实际输入状态
        uint16_t input_states = pcf8575.read16();
        
        // 恢复原始输出状态（不影响其他引脚）
        pcf8575.write16(pcf8575_pin_states);
        
        return (input_states & (1 << pcf_pin)) ? HIGH : LOW;
    }
}

void PCF8575_analogWrite(uint8_t pin, uint8_t value) {
    if (!IS_PCF8575_VIRTUAL_PIN(pin)) {
        return;
    }
    
    uint8_t pcf_pin = VIRTUAL_TO_PCF8575_PIN(pin);
    
    // 缓存PWM值
    pcf8575_pwm_values[pcf_pin] = value;
    
    // 简单的PWM模拟：根据PWM值设置引脚状态
    // 注意：这是软件模拟的PWM，精度有限
    static uint32_t last_pwm_time = 0;
    static uint8_t pwm_counter = 0;
    
    uint32_t current_time = millis();
    if (current_time - last_pwm_time > 1) { // 约1kHz PWM频率
        last_pwm_time = current_time;
        pwm_counter++;
        
        if (value > pwm_counter) {
            PCF8575_digitalWrite(pin, HIGH);
        } else {
            PCF8575_digitalWrite(pin, LOW);
        }
    }
}

// Arduino标准函数重写，用于透明替换
void pinMode_PCF8575(uint8_t pin, uint8_t mode) {
    if (IS_PCF8575_VIRTUAL_PIN(pin)) {
        PCF8575_pinMode(pin, mode);
    } else {
        pinMode(pin, mode);
    }
}

void digitalWrite_PCF8575(uint8_t pin, uint8_t value) {
    if (IS_PCF8575_VIRTUAL_PIN(pin)) {
        PCF8575_digitalWrite(pin, value);
    } else {
        digitalWrite(pin, value);
    }
}

int digitalRead_PCF8575(uint8_t pin) {
    if (IS_PCF8575_VIRTUAL_PIN(pin)) {
        return PCF8575_digitalRead(pin);
    } else {
        return digitalRead(pin);
    }
}

void analogWrite_PCF8575(uint8_t pin, uint8_t value) {
    if (IS_PCF8575_VIRTUAL_PIN(pin)) {
        PCF8575_analogWrite(pin, value);
    } else {
        analogWrite(pin, value);
    }
}