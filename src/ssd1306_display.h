#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// ============================
// OLED硬件配置
// ============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDR 0x3C
#define OLED_RESET U8X8_PIN_NONE

// ============================
// 按键定义
// ============================
#define KEY_FORWARD 2
#define KEY_CONFIRM 15
#define KEY_BACKWARD 3

// ============================
// 动画配置（优化响应速度）
// ============================
#define CURSOR_MOVE_DURATION 100          // 光标移动动画时长(ms) - 加快
#define SCROLL_ANIMATION_DURATION 150     // 菜单滚动动画时长(ms) - 加快
#define EASING_TYPE 0                     // 缓动类型: 0=线性, 1=EaseOutCubic, 2=EaseOutQuart

// ============================
// 菜单显示配置
// ============================
#define MENU_VISIBLE_ITEMS 3              // 中间可见选项数
#define MENU_TOTAL_DISPLAY_ROWS 5         // 总显示行数（含固定顶底）
#define MENU_ITEM_HEIGHT 12               // 每行高度(像素)
#define MENU_START_Y 23                   // 菜单起始Y坐标（留给标题）

// ============================
// 设置菜单状态
// ============================
enum SETUP_STATE {
    SETUP_NONE,
    SETUP_MAIN,
    SETUP_MOTOR_SELECT,
    SETUP_PULLBACK,
    SETUP_SLEEP,
    SETUP_DEBUG_MOTOR_SELECT,  // 电机选择菜单
    SETUP_DEBUG_CONTROL,        // 电机调试控制
    SETUP_CHANNEL_STATUS,       // 新增：通道在线/离线设置
    SETUP_AMS_COUNT,          // 新增：AMS数量设置
    SETUP_ABOUT                 // 关于页面
};

// ============================
// 显示数据结构体
// ============================
typedef struct {
    char extruder_state[16][2];
    bool extruder_online[16];
    float extruder_length[16];
    int current_slot;
    int current_page;
    bool ams_auto_mode;
    bool as5600_ok;
    bool motor_running;
} DisplayData;

// ============================
// 全局变量声明
// ============================
extern int PULLBACK_TIMEOUT[16];
extern int SLEEP_TIMEOUT_MIN;
extern bool SCREEN_ALWAYS_ON;
extern int AMS_COUNT;  // AMS数量

// ============================
// 函数声明
// ============================
void ssd1306_init();
void ssd1306_update_extruder_data();
void ssd1306_refresh_screen();
void check_microswitch_status();