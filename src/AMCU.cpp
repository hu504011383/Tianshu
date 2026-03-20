/**
 * AMCU.cpp - AMCU主控单元实现
 * 
 * 功能说明：
 * 1. 电机控制：实现4个电机的正转（进料）和反转（退料）控制
 * 2. 传感器模拟：模拟AS5600角度传感器数据
 * 3. 通信协议：与Bambu打印机进行通信
 * 4. 状态管理：管理耗材状态和运动状态
 */

#include "AMCU.h"
#include "Motor.h"
#include <limits>
#include <cmath>
#undef abs
#include <random>
#include <cstdlib>
#include "CRC.h"
#include <Adafruit_NeoPixel.h>
#include "PCF8575_PinMap.h"



// 硬件参数定义
#define max_filament_num 16        // 最大耗材数量
#define max_ams_num 4              // 最大AMS数量
#define WHEEL_D 4.5                // 轮子直径
#define SERVO_DELAY 100            // 伺服电机延迟时间(ms)
#define PULL_BACK_METER 200        // 退料距离
#define LED_BLINK_INTERVAL 500     // LED闪烁间隔
#define SERVO_COUNT 4              // 伺服电机数量
#define AS5600_PI 3.14159265358979323846  // π值

// 微动开关引脚定义（与ssd1306_display.cpp保持一致）
const int MICROSWITCH_PINS[8] = {26, 27, 28, 29, 57, 56, 55, 54};  // 通道1-8的微动开关引脚

// PCF8575地址

#define PCF8575_ADDR 0x20
PCF8575 pcf8575(PCF8575_ADDR);
//extern short pcf8575Num;

extern int PULLBACK_TIMEOUT[16];  // 退料超时时间数组

// AS5600模拟变量
int16_t as5600_current_angle[4] = {0, 0, 0, 0};  // 当前角度值
int16_t as5600_last_angle[4] = {0, 0, 0, 0};     // 上一次角度值
uint32_t as5600_last_time[4] = {0, 0, 0, 0};     // 上一次时间戳
float as5600_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 速度值

// RGB LED配置
#define PIN 16                    // RGB LED引脚
#define NUMPIXELS 1               // LED数量
Adafruit_NeoPixel RGB_2812(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ============================
//  电机引脚配置
//  说明：虽然定义了16个电机的引脚，但实际使用前4个
//  5-8号电机使用PCF8575虚拟引脚50-65
// ============================
const int motorPins[16][2] = {
    {11, 12},  // 1号电机: GPIO11, GPIO12
    {9, 10},   // 2号电机: GPIO9, GPIO10
    {7, 8},    // 3号电机: GPIO7, GPIO8
    {13, 14},  // 4号电机: GPIO13, GPIO14
    {63, 62},  // 5号电机: PCF8575引脚0和1（虚拟引脚GP50和GP51）
    {65, 64},  // 6号电机: PCF8575引脚2和3（虚拟引脚GP52和GP53）
    {59, 58},  // 7号电机: PCF8575引脚4和5（虚拟引脚GP54和GP55）
    {61, 60},  // 8号电机: PCF8575引脚6和7（虚拟引脚GP56和GP57）
    {80, 81},  // 9号电机（预留）: PCF8575引脚8和9（虚拟引脚GP58和GP59）
    {82, 83},  // 10号电机（预留）: PCF8575引脚10和11（虚拟引脚GP60和GP61）
    {84, 85},  // 11号电机（预留）: PCF8575引脚12和13（虚拟引脚GP62和GP63）
    {86, 87},  // 12号电机（预留）: PCF8575引脚14和15（虚拟引脚GP64和GP65）
    {88, 89},    // 13号电机（预留）
    {90, 91},    // 14号电机（预留）
    {92, 93},    // 15号电机（预留）
    {94, 95},    // 16号电机（预留）
};

// 微动开关引脚
const int switchPin = 0;            // 微动开关引脚

// 全局变量
uint32_t last_toggle_time = 0;      // 上次切换时间
bool if_as5600_init = false;        // AS5600初始化状态
char last_num = -1;                 // 上次耗材编号
unsigned long last_num_change_time = 0; // 上次耗材变化时间
unsigned long last_pullback_time = 0;   // 上次退料时间
bool execute_motion = true;         // 是否执行运动
bool last_execute_motion = false;   // 上次运动状态
int last_cmd = BambuBus_package_ERROR;  // 上次命令
int last_filament_num = 255;        // 上次耗材编号
float last_meters[max_filament_num];    // 上次耗材长度
float last_pullback_meters[max_filament_num]; // 上次退料长度
float last_action[max_filament_num];    // 上次动作
_filament_motion_state_set act;     // 当前动作状态
bool ams_enable = true;             // AMS使能
float target = 0;                   // 目标值
bool enable_as5600 = true;          // AS5600使能
int32_t now_distance = 0;           // 当前距离
bool is_need_pull_back = false;     // 是否需要退料
int now_filament_num_need_pull_back = 255; // 需要退料的耗材编号

// 随机数生成器（用于模拟传感器数据）
std::mt19937 generator(12345);
std::normal_distribution<float> distribution(0.0055f, 0.00083f);

// 微动开关状态
int switchState = 0;                // 当前开关状态
int lastSwitchState = 0;            // 上次开关状态

// 电机对象数组（16个电机，实际使用前4个）
Motor motors[16] = {
    Motor(motorPins[0][0], motorPins[0][1]), // 1号电机
    Motor(motorPins[1][0], motorPins[1][1]), // 2号电机
    Motor(motorPins[2][0], motorPins[2][1]), // 3号电机
    Motor(motorPins[3][0], motorPins[3][1]), // 4号电机
    Motor(motorPins[4][0], motorPins[4][1]), // 5号电机（预留）
    Motor(motorPins[5][0], motorPins[5][1]), // 6号电机（预留）
    Motor(motorPins[6][0], motorPins[6][1]), // 7号电机（预留）
    Motor(motorPins[7][0], motorPins[7][1]), // 8号电机（预留）
    Motor(motorPins[8][0], motorPins[8][1]), // 9号电机（预留）
    Motor(motorPins[9][0], motorPins[9][1]), // 10号电机（预留）
    Motor(motorPins[10][0], motorPins[10][1]), // 11号电机（预留）
    Motor(motorPins[11][0], motorPins[11][1]), // 12号电机（预留）
    Motor(motorPins[12][0], motorPins[12][1]), // 13号电机（预留）
    Motor(motorPins[13][0], motorPins[13][1]), // 14号电机（预留）
    Motor(motorPins[14][0], motorPins[14][1]), // 15号电机（预留）
    Motor(motorPins[15][0], motorPins[15][1]), // 16号电机（预留）
};

// ==============================
//  RGB控制函数
// ==============================
void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    RGB_2812.setPixelColor(0, RGB_2812.Color(r, g, b));
    RGB_2812.setBrightness(255); // 固定最亮，无PWM
    RGB_2812.show();
}

// RGB状态定义（匹配电机动作）
#define RGB_RED    set_rgb(5,0,0)  // 停止/故障/取消
#define RGB_GREEN  set_rgb(0,5,0)  // 进料/退料动作中
#define RGB_BLUE   set_rgb(0,0,5)  // 待机/正常

/**
 * @brief 初始化IO引脚
 * 功能：设置微动开关和电机引脚的模式
 */
void init_io_pins() {
    // 微动开关引脚：输入模式+内部上拉
    pinMode(switchPin, INPUT_PULLUP);
    
    // 初始化自动模式微动开关引脚（通道1-8）
    for (int i = 0; i < 8; i++) {
        int pin = MICROSWITCH_PINS[i];
        if (IS_PCF8575_VIRTUAL_PIN(pin)) {
            // PCF8575虚拟引脚使用专用函数设置为输入模式
            PCF8575_pinMode(pin, INPUT);
        } else {
            // Pico物理引脚使用标准函数
            pinMode(pin, INPUT_PULLUP);
        }
    }

    // 初始化所有电机引脚
    for (int i = 0; i < 4*max_ams_num; i++) {
        // 使用PCF8575虚拟引脚处理函数
        pinMode_PCF8575(motorPins[i][0], OUTPUT); // 正转（进料）引脚
        pinMode_PCF8575(motorPins[i][1], OUTPUT); // 反转（退料）引脚
        digitalWrite_PCF8575(motorPins[i][0], LOW);
        digitalWrite_PCF8575(motorPins[i][1], LOW);
    }
}

/**
 * @brief 电机控制核心函数
 * @param motorNum 电机编号(0~15) → 对应1~16号电机
 * @param cmd 动作指令：1=正转(进料), -1=反转(退料), 0=停止
 * 功能：控制电机的正转、反转和停止
 */
void motorCmd(int motorNum, int cmd) {
    // 校验电机编号合法性，超出范围时取模
    if (motorNum < 0 || motorNum >= (16)) {
        motorNum = motorNum%(16);
    }

    // 8833驱动控制逻辑（严格匹配进退料）
    if (cmd == 0) {
        motors[motorNum].stop();          // 停止电机
        RGB_BLUE;                         // 待机→蓝灯
    } else if (cmd > 0) {
        motors[motorNum].forward();       // 正转→进料
        RGB_GREEN;                        // 动作中→绿灯
    } else {
        motors[motorNum].backforward();   // 反转→退料
        RGB_GREEN;                        // 动作中→绿灯
    }
}

/**
 * @brief 设置电机PWM占空比
 * @param motorNum 电机编号(0~3)
 * @param dutyCycle 占空比(0-255)
 * 功能：设置电机的速度
 */
void setMotorDutyCycle(int motorNum, uint8_t dutyCycle) {
    if (motorNum < 0 || motorNum >= 16) return;
    motors[motorNum].setDutyCycle(dutyCycle);
}

/**
 * @brief 初始化AS5600传感器（模拟版）
 * 功能：初始化模拟的AS5600角度传感器
 */
void AS5600_init() {
    if_as5600_init = true;
    generator.seed(millis());
    randomSeed(millis());
    
    // 初始化角度和时间戳
    for (int i = 0; i < 4; i++) {
        as5600_current_angle[i] = rand() % 4096;
        as5600_last_angle[i] = as5600_current_angle[i];
        as5600_last_time[i] = millis();
        as5600_speed[i] = 0.0f;
    }
    
    if (if_as5600_init) printf("AS5600 初始化成功\n");
    else printf("AS5600 初始化失败\n");
}

/**
 * @brief 获取模拟的AS5600距离数据
 * @return 模拟的距离值
 * 功能：生成模拟的距离数据，用于测试
 */
float AS5600_get_distance_E() {
    if (!enable_as5600) return 0;
    
    // 使用正态分布生成随机值（均值0.0055m，标准差0.00083m）
    float random_value = distribution(generator);
    
    // 截断到[0.003, 0.008]范围
    if (random_value < 0.003f) random_value = 0.003f;
    if (random_value > 0.008f) random_value = 0.008f;
    
    return random_value;
}

/**
 * @brief 获取模拟的AS5600原始角度值
 * @param channel 通道号（0-3）
 * @return 模拟的角度值
 * 功能：生成模拟的角度数据，用于测试
 */
uint16_t AS5600_get_raw_angle(uint8_t channel) {
    if (!enable_as5600 || channel >= 4) return 0;
    
    // 模拟角度值的连续变化
    uint32_t now = millis();
    uint32_t time_diff = now - as5600_last_time[channel];
    
    // 计算角度增量（模拟电机转动）
    int16_t angle_increment = (rand() % 10) - 5; // -5到4的随机增量
    
    // 更新角度值，确保在0-4095之间循环
    as5600_current_angle[channel] = (as5600_current_angle[channel] + angle_increment) % 4096;
    if (as5600_current_angle[channel] < 0) {
        as5600_current_angle[channel] += 4096;
    }
    
    // 计算速度
    int32_t angle_diff = as5600_current_angle[channel] - as5600_last_angle[channel];
    if (angle_diff >  2048) angle_diff -= 4096;
    if (angle_diff < -2048) angle_diff += 4096;
    
    if (time_diff > 0) {
        float dist_mm = (float)angle_diff * AS5600_MM_PER_CNT;
        as5600_speed[channel] = dist_mm / (float)time_diff * 1000.0f; // mm/s
    }
    
    // 更新最后角度和时间
    as5600_last_angle[channel] = as5600_current_angle[channel];
    as5600_last_time[channel] = now;
    
    return as5600_current_angle[channel];
}

/**
 * @brief 获取模拟的AS5600磁场状态
 * @param channel 通道号（0-3）
 * @return 磁场状态
 * 功能：返回模拟的磁场状态，始终为正常
 */
AS5600_magnet_stu AS5600_get_magnet_stu(uint8_t channel) {
    if (!enable_as5600 || channel >= 4) return AS5600_offline;
    
    // 始终返回正常磁场状态
    return AS5600_normal;
}

/**
 * @brief 检查AS5600是否在线
 * @param channel 通道号（0-3）
 * @return 是否在线
 * 功能：返回模拟的在线状态，始终为在线
 */
bool AS5600_is_online(uint8_t channel) {
    if (!enable_as5600 || channel >= 4) return false;
    
    // 始终返回在线状态
    return true;
}

/**
 * @brief 获取模拟的AS5600速度
 * @param channel 通道号（0-3）
 * @return 速度值
 * 功能：返回模拟的速度数据
 */
float AS5600_get_speed(uint8_t channel) {
    if (!enable_as5600 || channel >= 4) return 0.0f;
    return as5600_speed[channel];
}

/**
 * @brief AMCU系统初始化
 * 功能：初始化IO引脚、传感器、通信等
 */
void AMCU_init() {
    // 初始化PCF8575虚拟引脚映射
    PCF8575_init();
    
    init_io_pins();      // 初始化IO引脚
    AS5600_init();       // 初始化AS5600传感器
    
    RGB_BLUE; // 上电默认蓝灯（待机）
    if (!if_as5600_init) RGB_RED; // 传感器异常→红灯

    BambuBus_init();     // 初始化BambuBus通信
    releaseAllPos();     // 释放所有位置
    delay(500);          // 延迟500ms

    // 初始化耗材长度记录
    for (int i = 0; i < max_filament_num; i++)
        last_meters[i] = std::numeric_limits<float>::lowest();

    // 通道状态会从Flash加载，不需要默认设置
    // 但为了确保设备能被正确识别，默认设置所有通道为在线状态
    for (int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++){
           set_filament_online(i, j, true);
        }
    }
        
}

/**
 * @brief 微动开关中断处理
 * 功能：当微动开关被触发时，控制选中的电机进料200ms
 */
void switchInterrupt() {
    switchState = digitalRead(switchPin);
    if (switchState != lastSwitchState) {
        if (switchState == LOW) {
            char n = get_now_filament_num();
            motorCmd(n, 1); // 正转→进料
            RGB_GREEN;      // 绿灯
            delay(200);     // 进料200ms
            motorCmd(n, 0); // 停止
            RGB_BLUE;       // 蓝灯
        }
        delay(50); // 防抖
        lastSwitchState = switchState;
    }
}

// 空实现：保留接口兼容性
void selectOnePos(char num) { RGB_BLUE; }
void releaseAllPos() {}
void AMCU_bus_deal_set_motion_res(uint8_t *data, int len) {}

/**
 * @brief AMCU主运行逻辑
 * 功能：处理耗材状态、更新耗材长度、运行通信和调试逻辑
 */
void AMCU_run() {
    bool if_count_meters = true;
    float d = AS5600_get_distance_E();
    static int now_f = 255;
    int x = get_now_filament_num();

    // 耗材槽切换处理
    if (now_f != x) {
        now_f = x;
        Bambubus_set_need_to_save(); // 设置需要保存
    }

    // 处理耗材运动状态（匹配进退料）
    auto state = get_filament_motion(now_f);
    if (state == need_pull_back) {
        // 需要退料
        if (last_action[now_f] != need_pull_back) {
            last_action[now_f] = need_pull_back;
            last_pullback_meters[now_f] = get_filament_meters(now_f);
            is_need_pull_back = true;
            now_filament_num_need_pull_back = now_f;
            last_pullback_time = millis();
            RGB_GREEN; // 退料中→绿灯
        }
    } else if (state == need_send_out) {
        // 需要进料
        last_action[now_f] = need_send_out;
        RGB_GREEN; // 进料中→绿灯
    } else if (state == waiting) {
        // 待机
        last_action[now_f] = waiting;
        RGB_BLUE; // 待机→蓝灯
    }

    // 更新耗材长度
    if (if_count_meters || ams_enable == false) {
        if(state == need_send_out){
            d = -d; // 进料时长度减少
        }
        add_filament_meters(now_f, d);
    }

    // 运行通信和调试逻辑
    int stu = -1;
    if (ams_enable) stu = BambuBus_run();

    AMCU_motion();   // 处理电机运动
}

/**
 * @brief 电机运动逻辑
 * 功能：根据当前状态控制电机的运动
 */
void AMCU_motion() {
    char cur = get_now_filament_num();
    // 耗材槽切换防抖
    if (cur != last_num) {
        last_num = cur;
        last_num_change_time = millis();
        execute_motion = false;
    }
    if (!execute_motion && millis()-last_num_change_time >= SERVO_DELAY)
        execute_motion = true;

    // 处理退料超时
    if (is_need_pull_back) {
        // 获取当前退料电机的超时时间（默认为7秒）
        int timeout_seconds = 7;  // 默认7秒
        if (now_filament_num_need_pull_back >= 0 && now_filament_num_need_pull_back < 4*max_ams_num) {
            timeout_seconds = PULLBACK_TIMEOUT[now_filament_num_need_pull_back];
        }
        
        if (millis()-last_pullback_time >= (timeout_seconds * 1000)) {
            // 保存当前退料的通道号
            int pull_back_channel = now_filament_num_need_pull_back;
            
            motorCmd(pull_back_channel, 0); // 停止退料
            is_need_pull_back = false;
            now_filament_num_need_pull_back = 255;
            
            // 更新状态为waiting
            if (ams_enable && pull_back_channel >= 0) {
                set_filament_motion(pull_back_channel, waiting);
            }
            RGB_BLUE; // 待机→蓝灯
        } else {
            motorCmd(now_filament_num_need_pull_back, -1); // 继续退料（反转）
            RGB_GREEN; // 退料中→绿灯
        }
    }

    // 更新当前动作状态
    if (ams_enable) act = get_filament_motion(cur);

    // 处理不同运动状态（严格匹配进退料）
    switch (act) {
        case need_pull_back:
            // 待退料→绿灯
            RGB_GREEN; 
            break;
        case need_send_out:
            // 待进料：执行进料（正转）
            if (execute_motion && is_filament_online(cur)) {
                motorCmd(cur, 1); // 正转→进料
                RGB_GREEN;
            } else {
                motorCmd(cur, 0); // 停止
                RGB_BLUE;
            }
            break;
        case waiting:
            // 待机：停止非退料电机，处理微动开关
            if (cur != now_filament_num_need_pull_back) motorCmd(cur, 0);
            if (!is_need_pull_back) switchInterrupt();
            RGB_BLUE;
            break;
        case act_send_mm:
            // 执行进料（正转）
            if (last_pullback_meters[cur] - get_filament_meters(cur) > target) {
                motorCmd(cur, 0); // 达到目标→停止
                RGB_BLUE; 
                act = waiting;
            } else if (execute_motion && is_filament_online(cur)) {
                motorCmd(cur, 1); // 继续进料（正转）
                RGB_GREEN;
            } else {
                motorCmd(cur, 0); // 停止
                RGB_BLUE;
            }
            break;
        case act_pull_mm:
            // 执行退料（反转）
            if (get_filament_meters(cur) - last_pullback_meters[cur] > target) {
                motorCmd(cur, 0); // 达到目标→停止
                RGB_BLUE; 
                act = waiting;
            } else if (execute_motion && is_filament_online(cur)) {
                motorCmd(cur, -1); // 继续退料（反转）
                RGB_GREEN;
            } else {
                motorCmd(cur, 0); // 停止
                RGB_BLUE;
            }
            break;
        case cancel:
        case release_all:
            // 取消/释放→停止电机+红灯
            motorCmd(cur, 0); 
            RGB_RED; 
            act = waiting;
            break;
        default:
            // 默认→处理微动开关+蓝灯
            switchInterrupt(); 
            RGB_BLUE; 
            break;
    }
}