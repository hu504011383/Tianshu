#include "ssd1306_display.h"
#include <string.h>
#include <math.h>
#include "AMCU.h"
#include "settings_storage.h"
#include "PCF8575.h"
#include "PCF8575_PinMap.h"
// ============================
// u8g2显示对象
// ============================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RESET);

// ============================
// 系统配置变量
// ============================
//short pcf8575Num = 0; //初始化为0·未连接
int PULLBACK_TIMEOUT[16] = {7, 7, 7, 7, 7,7,7,7,7,7,7,7,7,7,7,7};
int SLEEP_TIMEOUT_MIN = 15;
bool SCREEN_ALWAYS_ON = false;
int CHANNEL_MODE[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 通道状态模式（0=离线, 1=在线, 2=自动）
int AMS_COUNT = 4;  // AMS数量（1-4）

// ============================
// 微动开关引脚定义
// ============================
// 通道1-4使用26-29引脚，通道5-8使用58-61引脚
const int MICROSWITCH_PINS[8] = {26, 27, 28, 29, 57, 56, 55, 54};  // 通道1-8的微动开关引脚

// 过程跟踪变量
bool in_feeding_process[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
bool in_pulling_process[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
unsigned long action_complete_time[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool prev_extruder_state_feeding[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
bool prev_extruder_state_pulling[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
#define ACTION_COMPLETE_DELAY 500
const int KEY_DEBOUNCE = 200;
const int LONG_PRESS_DELAY = 600; // 长按阈值
#define SLEEP_TIMEOUT (SCREEN_ALWAYS_ON ? 0 : (SLEEP_TIMEOUT_MIN * 60 * 1000))

unsigned long last_action_time = 0;
bool screen_sleep = false;
unsigned long key_backward_press_time = 0;
bool key_backward_pressed = false;

// ============================
// 调试控制变量（新增）
// ============================
int debug_selected_motor = 0;      // 当前选中的电机(0-15)
int debug_motor_state[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 0=停止, 1=进料, -1=退料
bool debug_control_initialized = false;

// ============================
// 通道状态设置变量（新增）
// ============================
int channel_status_selected = 0;   // 当前选中的通道(0-15)
bool channel_status_initialized = false;

// ============================
// 检查微动开关状态并更新自动模式下的通道状态
// ============================
void check_microswitch_status() {
    for (int i = 0; i < 8; i++) {  // 只处理1-8通道
        if (CHANNEL_MODE[i] == CHANNEL_MODE_AUTO) {
            int pin = MICROSWITCH_PINS[i];
            // 根据引脚类型使用正确的读取函数
            bool switchState;
            if (IS_PCF8575_VIRTUAL_PIN(pin)) {
                switchState = PCF8575_digitalRead(pin);
            } else {
                switchState = digitalRead(pin);
            }
            set_filament_online(i / 4, i % 4, switchState);
        }
    }
}

// ============================
// 菜单系统变量
// ============================
int cursorTargetRow = 0;

// 关于页面变量
int about_page = 0; // 当前页码

// 关于页面的统一信息数组
const char* about_info[] = {
    "版本:v2.0.1",
    "定制联系QQ",
    "504011383",
    "QQ群:981655616",
    "特别鸣谢:",
    "Dxt、林克、道明寺",
    "Yeah、facetruth 、",
    "通信协议: PJARCZAK",
    "功能开发: 鱼丸粗面",
    "功能介绍:",
    "- 支持最多16通道",
    "- 硬件支持8通道",
    "- 支持设置通道状态和退料时间",
    "- 模拟AS5600_来自Dxt",
    "- 模拟4个AMS级联",
    "- 支持自定义AMS数量",
    "- 支持电机拖机调试"
};
const int total_about_lines = sizeof(about_info) / sizeof(about_info[0]);
const int about_lines_per_page = 3;

// ============================
// 菜单系统
// ============================
SETUP_STATE setup_state = SETUP_NONE;
SETUP_STATE last_drawn_state = SETUP_NONE;  // 记录上次绘制的状态，用于检测状态变化

struct MenuItem {
    const char* label;
    const char* value;
    bool hasSubMenu;
};

#define MAX_MENU_ITEMS 16
MenuItem currentMenu[MAX_MENU_ITEMS];
int menuItemCount = 0;
int menuSelectedIndex = 0;
int menuVisibleOffset = 0;

int setup_selected = 0;
int motor_select_index = 0;
unsigned long last_key_time = 0;

// 菜单初始化标志 - 每个状态单独跟踪
bool main_menu_initialized = false;
bool motor_select_initialized = false;

// ============================
// AMCU全局变量声明
// ============================
extern bool ams_enable;
extern bool enable_as5600;
extern bool if_as5600_init;
extern bool execute_motion;
extern _filament_motion_state_set act;
extern unsigned long last_pullback_time;
extern bool is_need_pull_back;
extern int now_filament_num_need_pull_back;

// ============================
// 显示数据
// ============================
DisplayData display_data = {0};
bool displaymode = true;

// ============================
// 启动图片数据 (128x64)
// ============================
const uint8_t startup_image[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x10, 0x00, 0x04, 0xff, 0xf8, 0x18, 0x7f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xf8, 0x18, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x18, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x7e, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x7e, 0xcc, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x18, 0xcc, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1c, 0xe6, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x0c, 0x1f, 0xff, 0xfc, 0x3c, 0xc7, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xfc, 0x3e, 0xc3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x3b, 0xc3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x78, 0xc3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x58, 0xe7, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x00, 0xd8, 0xe6, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe3, 0x80, 0x98, 0xcc, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1, 0xc0, 0x18, 0xd8, 0x61, 0xc8, 0x9e, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0xf0, 0x18, 0xc0, 0x01, 0x6c, 0xb0, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x7c, 0x18, 0xe0, 0x03, 0x35, 0x9c, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x1c, 0x18, 0xff, 0xfa, 0x2a, 0x46, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x04, 0x18, 0x7e, 0x01, 0xc8, 0xdc, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc8, 0x1c, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x03, 0x80, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ============================
// 菜单系统函数
// ============================
void clearMenu() {
    menuItemCount = 0;
    menuSelectedIndex = 0;
    menuVisibleOffset = 0;
}

void addMenuItem(const char* label, const char* value = nullptr, bool hasSub = false) {
    if (menuItemCount >= MAX_MENU_ITEMS) return;
    currentMenu[menuItemCount].label = label;
    currentMenu[menuItemCount].value = value;
    currentMenu[menuItemCount].hasSubMenu = hasSub;
    menuItemCount++;
}

void setMenuSelection(int index) {
    if (index < 0 || index >= menuItemCount) return;
    
    menuSelectedIndex = index;
    
    // 计算新位置是否需要滚动
    if (index < menuVisibleOffset) {
        // 需要向上滚动
        menuVisibleOffset = index;
        cursorTargetRow = 0;
    } else if (index >= menuVisibleOffset + MENU_VISIBLE_ITEMS) {
        // 需要向下滚动
        menuVisibleOffset = index - MENU_VISIBLE_ITEMS + 1;
        cursorTargetRow = MENU_VISIBLE_ITEMS - 1;
    } else {
        // 在可见范围内，只移动光标
        cursorTargetRow = index - menuVisibleOffset;
    }
}

void menuNext() {
    if (menuSelectedIndex < menuItemCount - 1) {
        setMenuSelection(menuSelectedIndex + 1);
    } else {
        // 循环到第一项
        setMenuSelection(0);
    }
}

void menuPrev() {
    if (menuSelectedIndex > 0) {
        setMenuSelection(menuSelectedIndex - 1);
    } else {
        // 循环到最后一项
        setMenuSelection(menuItemCount - 1);
    }
}

const char* get_state_word(char symbol) {
    switch(symbol) {
        case '<': return "进料";
        case '>': return "退料";
        default:  return "就绪";
    }
}

bool key_pressed(int pin) {
    if (digitalRead(pin) == LOW && (millis() - last_key_time) > KEY_DEBOUNCE) {
        last_key_time = millis();
        return true;
    }
    return false;
}

bool is_any_moving() {
    for (int i=0; i<16; i++) {
        if (display_data.extruder_state[i][0] == '<' || 
            display_data.extruder_state[i][0] == '>') {
            return true;
        }
    }
    return false;
}

void ssd1306_init() {
    pinMode(KEY_FORWARD, INPUT_PULLUP);
    pinMode(KEY_CONFIRM, INPUT_PULLUP);
    pinMode(KEY_BACKWARD, INPUT_PULLUP);

    for (int i=0; i<16; i++) {
        display_data.extruder_state[i][0] = '-';
        display_data.extruder_state[i][1] = '\0';
        display_data.extruder_online[i] = false;
        display_data.extruder_length[i] = 0;
        in_feeding_process[i] = false;
        in_pulling_process[i] = false;
        action_complete_time[i] = 0;
        prev_extruder_state_feeding[i] = false;
        prev_extruder_state_pulling[i] = false;
    }
    display_data.current_slot = 0;
    display_data.current_page = 0;
    display_data.ams_auto_mode = true;
    display_data.as5600_ok = true;
    display_data.motor_running = false;

    display.begin();
    display.enableUTF8Print();
    
    // 显示启动图片
    display.clearBuffer();
    for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) {
            int byte_index = (y * 128) / 8 + x / 8;
            int bit_index = 7 - (x % 8);
            bool pixel = (startup_image[byte_index] >> bit_index) & 1;
            if (pixel) {
                display.drawPixel(x, y);
            }
        }
    }
    display.sendBuffer();
    
    // 延时5秒
    delay(5000);

    last_action_time = millis();
    screen_sleep = false;
    
    // 重置所有初始化标志
    main_menu_initialized = false;
    motor_select_initialized = false;
    channel_status_initialized = false;
    last_drawn_state = SETUP_NONE;
}

void update_screen_sleep() {
    if (SCREEN_ALWAYS_ON) {
        if (screen_sleep) {
            screen_sleep = false;
            display.setPowerSave(0);
        }
        return;
    }
    bool moving = is_any_moving();
    if (moving) {
        last_action_time = millis();
        if (screen_sleep) {
            screen_sleep = false;
            display.setPowerSave(0);
        }
    } else {
        if (!screen_sleep && (millis() - last_action_time >= SLEEP_TIMEOUT)) {
            screen_sleep = true;
            display.setPowerSave(1);
        }
    }
}

// ============================
// 菜单绘制函数
// ============================

void draw_setup_main() {
    // 检测是否需要重新初始化（状态变化或首次进入）
    if (last_drawn_state != SETUP_MAIN) {
        main_menu_initialized = false;
    }
    
    if (!main_menu_initialized) {
        clearMenu();
        static char sleepVal[16];
        addMenuItem("退料时间", nullptr, true);
   
        addMenuItem("屏幕休眠", nullptr, true);
        // 电机调试
        addMenuItem("电机调试", nullptr, true);
        // 通道状态设置（新增）
        addMenuItem("通道状态", nullptr, true);
        // AMS状态设置（新增）
        addMenuItem("AMS状态", nullptr, true);
        // 关于
        addMenuItem("关于", nullptr, true);
        
        // 恢复上次选中的位置
        if (setup_selected >= menuItemCount) setup_selected = 0;
        setMenuSelection(setup_selected);
        main_menu_initialized = true;
    }
    
    // 绘制框架
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setDrawColor(1);
    display.drawBox(0, 0, SCREEN_WIDTH, 12);
    display.setDrawColor(0);
    int titleX = (SCREEN_WIDTH - display.getStrWidth("设置菜单")) / 3;
    display.setCursor(titleX, 10);
    display.print("设置菜单");
    display.setDrawColor(1);
    display.drawHLine(0, 50, SCREEN_WIDTH);
    
    // 绘制菜单项
    for (int i = 0; i < MENU_VISIBLE_ITEMS; i++) {
        int itemIndex = menuVisibleOffset + i;
        if (itemIndex >= menuItemCount) break;
        
        bool isSelected = (itemIndex == menuSelectedIndex);
        int y = MENU_START_Y + i * MENU_ITEM_HEIGHT;
        
        // 绘制选中背景
        if (isSelected) {
            display.setDrawColor(1);
            display.drawBox(0, MENU_START_Y + cursorTargetRow * MENU_ITEM_HEIGHT - 10, SCREEN_WIDTH, MENU_ITEM_HEIGHT);
            display.setDrawColor(0);
        }
        
        display.setCursor(8, y);
        display.print(currentMenu[itemIndex].label);
        if (currentMenu[itemIndex].value != nullptr) {
            int valueX = SCREEN_WIDTH - display.getStrWidth(currentMenu[itemIndex].value) - 8;
            display.setCursor(valueX, y);
            display.print(currentMenu[itemIndex].value);
        }
        
        if (isSelected) {
            display.setDrawColor(1);
        }
    }
    
    // 绘制底部提示
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setCursor(0, 61);
    display.print("<上    确认进入   下>");
    
    display.sendBuffer();
    last_drawn_state = SETUP_MAIN;
}

void draw_motor_select() {
    // 检测是否需要重新初始化
    if (last_drawn_state != SETUP_MOTOR_SELECT) {
        motor_select_initialized = false;
    }
    
    if (!motor_select_initialized) {
        clearMenu();
        static char labels[16][16];
        static char values[16][16];
        for (int i = 0; i < 16; i++) {
            sprintf(labels[i], "电机%d", i + 1);
            sprintf(values[i], "%d秒", PULLBACK_TIMEOUT[i]);
            addMenuItem(labels[i], values[i], true);
        }
        // 恢复上次选中的位置
        if (motor_select_index >= menuItemCount) motor_select_index = 0;
        setMenuSelection(motor_select_index);
        motor_select_initialized = true;
    }
    
    // 更新动态值
    static char values[16][16];
    for (int i = 0; i < 16; i++) {
        sprintf(values[i], "%d秒", PULLBACK_TIMEOUT[i]);
        currentMenu[i].value = values[i];
    }
    
    // 绘制框架
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setDrawColor(1);
    display.drawBox(0, 0, SCREEN_WIDTH, 12);
    display.setDrawColor(0);
    int titleX = (SCREEN_WIDTH - display.getStrWidth("选择电机(1-16)")) / 2;
    display.setCursor(titleX, 10);
    display.print("选择电机(1-16)");
    display.setDrawColor(1);
    display.drawHLine(0, 50, SCREEN_WIDTH);
    
    // 绘制菜单项
    for (int i = 0; i < MENU_VISIBLE_ITEMS; i++) {
        int itemIndex = menuVisibleOffset + i;
        if (itemIndex >= menuItemCount) break;
        
        bool isSelected = (itemIndex == menuSelectedIndex);
        int y = MENU_START_Y + i * MENU_ITEM_HEIGHT;
        
        if (isSelected) {
            display.setDrawColor(1);
            display.drawBox(0, MENU_START_Y + cursorTargetRow * MENU_ITEM_HEIGHT - 10, SCREEN_WIDTH, MENU_ITEM_HEIGHT);
            display.setDrawColor(0);
        }
        
        display.setCursor(8, y);
        display.print(currentMenu[itemIndex].label);
        if (currentMenu[itemIndex].value != nullptr) {
            int valueX = SCREEN_WIDTH - display.getStrWidth(currentMenu[itemIndex].value) - 8;
            display.setCursor(valueX, y);
            display.print(currentMenu[itemIndex].value);
        }
        
        if (isSelected) {
            display.setDrawColor(1);
        }
    }
    
    // 绘制滚动条
    if (menuItemCount > MENU_VISIBLE_ITEMS) {
        int contentHeight = MENU_VISIBLE_ITEMS * MENU_ITEM_HEIGHT;
        int scrollBarHeight = contentHeight * MENU_VISIBLE_ITEMS / menuItemCount;
        int scrollBarY = MENU_START_Y - 10 + (menuVisibleOffset * contentHeight / menuItemCount);
        display.drawFrame(SCREEN_WIDTH - 4, MENU_START_Y - 10, 4, contentHeight);
        display.drawBox(SCREEN_WIDTH - 3, scrollBarY, 2, scrollBarHeight);
    }
    
    // 绘制底部提示
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setCursor(0, 62);
    display.print("<上    确认进入   下>");
    
    display.sendBuffer();
    last_drawn_state = SETUP_MOTOR_SELECT;
}

void draw_setup_pullback() {
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    
    char title[32];
    sprintf(title, "电机%d退料时间", motor_select_index + 1);
    display.setCursor((SCREEN_WIDTH - display.getStrWidth(title)) / 2, 10);
    display.print(title);
    
    display.setCursor(0, 23);
    display.print("当前: ");
    display.print(PULLBACK_TIMEOUT[motor_select_index]);
    display.print("秒 (范围3-30)");
    
    display.setCursor(0, 62);
    display.print("<减      保存     加>");
    display.sendBuffer();
    
    last_drawn_state = SETUP_PULLBACK;
}

void draw_setup_sleep() {
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    
    display.setCursor(0, 10);
    display.print("休眠时间设置");
    
    display.setCursor(0, 23);
    display.print("当前: ");
    if (SCREEN_ALWAYS_ON) {
        display.print("常亮");
    } else {
        display.print(SLEEP_TIMEOUT_MIN);
        display.print("分");
    }
    display.print(" (范围1-15)");
    
    display.setCursor(0, 62);
    if (SCREEN_ALWAYS_ON) {
        display.print("<返回           常亮>");
    } else {
        display.print("<减      保存     加>");
    }
    
    display.sendBuffer();
    
    last_drawn_state = SETUP_SLEEP;
}

void draw_setup_ams_count() {
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    
    display.setCursor(0, 10);
    display.print("AMS数量设置");
    
    display.setCursor(0, 23);
    display.print("当前: ");
    display.print(AMS_COUNT);
    display.print(" (范围1-4)");
    
    display.setCursor(0, 62);
    display.print("<减      保存     加>");
    display.sendBuffer();
    
    last_drawn_state = SETUP_AMS_COUNT;
}

// ============================
// 新增：电机调试控制页面
// ============================
// 电机选择菜单（用于电机调试）
void draw_debug_motor_select() {
    // 检测是否需要重新初始化
    if (last_drawn_state != SETUP_DEBUG_MOTOR_SELECT) {
        debug_control_initialized = false;
    }
    
    if (!debug_control_initialized) {
        clearMenu();
        static char labels[16][16];
        for (int i = 0; i < 16; i++) {
            sprintf(labels[i], "电机%d", i + 1);
            addMenuItem(labels[i], nullptr, true);
        }
        // 恢复上次选中的位置
        if (debug_selected_motor >= menuItemCount) debug_selected_motor = 0;
        setMenuSelection(debug_selected_motor);
        debug_control_initialized = true;
    }
    
    // 绘制框架
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setDrawColor(1);
    display.drawBox(0, 0, SCREEN_WIDTH, 12);
    display.setDrawColor(0);
    int titleX = (SCREEN_WIDTH - display.getStrWidth("选择电机(1-16)")) / 2;
    display.setCursor(titleX, 10);
    display.print("选择电机(1-16)");
    display.setDrawColor(1);
    display.drawHLine(0, 50, SCREEN_WIDTH);
    
    // 绘制菜单项
    for (int i = 0; i < MENU_VISIBLE_ITEMS; i++) {
        int itemIndex = menuVisibleOffset + i;
        if (itemIndex >= menuItemCount) break;
        
        bool isSelected = (itemIndex == menuSelectedIndex);
        int y = MENU_START_Y + i * MENU_ITEM_HEIGHT;
        
        if (isSelected) {
            display.setDrawColor(1);
            display.drawBox(0, MENU_START_Y + cursorTargetRow * MENU_ITEM_HEIGHT - 10, SCREEN_WIDTH, MENU_ITEM_HEIGHT);
            display.setDrawColor(0);
        }
        
        display.setCursor(8, y);
        display.print(currentMenu[itemIndex].label);
        
        if (isSelected) {
            display.setDrawColor(1);
        }
    }
    
    // 绘制滚动条
    if (menuItemCount > MENU_VISIBLE_ITEMS) {
        int contentHeight = MENU_VISIBLE_ITEMS * MENU_ITEM_HEIGHT;
        int scrollBarHeight = contentHeight * MENU_VISIBLE_ITEMS / menuItemCount;
        int scrollBarY = MENU_START_Y - 10 + (menuVisibleOffset * contentHeight / menuItemCount);
        display.drawFrame(SCREEN_WIDTH - 4, MENU_START_Y - 10, 4, contentHeight);
        display.drawBox(SCREEN_WIDTH - 3, scrollBarY, 2, scrollBarHeight);
    }
    
    // 绘制底部提示
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setCursor(0, 62);
    display.print("<上    确认进入   下>");
    
    display.sendBuffer();
    last_drawn_state = SETUP_DEBUG_MOTOR_SELECT;
}

// 电机调试控制界面
void draw_debug_control() {
    // 检测是否需要重新初始化
    if (last_drawn_state != SETUP_DEBUG_CONTROL) {
        debug_control_initialized = false;
    }
    
    if (!debug_control_initialized) {
        // 进入调试模式时，先停止所有电机
        for (int i = 0; i < 16; i++) {
            debug_motor_state[i] = 0;
            motorCmd(i, 0);  // 停止所有电机
        }
        debug_control_initialized = true;
    }
    
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    
    char title[32];
    sprintf(title, "电机%d调试控制", debug_selected_motor + 1);
    display.setCursor((SCREEN_WIDTH - display.getStrWidth(title)) / 2, 10);
    display.print(title);
    
    // 显示当前状态
    const char* state_str[] = {"停止", "进料", "退料"};
    int state_idx = debug_motor_state[debug_selected_motor] + 1;  // -1->0, 0->1, 1->2
    
    display.setCursor(0, 23);
    display.print("当前: ");
    display.print(state_str[state_idx]);
    
    // 操作提示
    display.setCursor(0, 62);
    display.print("<切换  进/停/退  返回>");
    
    display.sendBuffer();
    
    last_drawn_state = SETUP_DEBUG_CONTROL;
}

// ============================
// 新增：通道在线状态设置页面
// ============================
void draw_channel_status() {
    // 检测是否需要重新初始化
    if (last_drawn_state != SETUP_CHANNEL_STATUS) {
        channel_status_initialized = false;
    }
    
    if (!channel_status_initialized) {
        clearMenu();
        static char labels[16][16];
        for (int i = 0; i < 16; i++) {
            sprintf(labels[i], "通道%d", i + 1);
            addMenuItem(labels[i], nullptr, true);
        }
        // 恢复上次选中的位置
        if (channel_status_selected >= menuItemCount) channel_status_selected = 0;
        setMenuSelection(channel_status_selected);
        channel_status_initialized = true;
    }
    
    // 绘制框架
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setDrawColor(1);
    display.drawBox(0, 0, SCREEN_WIDTH, 12);
    display.setDrawColor(0);
    int titleX = (SCREEN_WIDTH - display.getStrWidth("选择通道(1-16)")) / 2;
    display.setCursor(titleX, 10);
    display.print("选择通道(1-16)");
    display.setDrawColor(1);
    display.drawHLine(0, 50, SCREEN_WIDTH);
    
    // 绘制菜单项
    for (int i = 0; i < MENU_VISIBLE_ITEMS; i++) {
        int itemIndex = menuVisibleOffset + i;
        if (itemIndex >= menuItemCount) break;
        
        bool isSelected = (itemIndex == menuSelectedIndex);
        int y = MENU_START_Y + i * MENU_ITEM_HEIGHT;
        
        if (isSelected) {
            display.setDrawColor(1);
            display.drawBox(0, MENU_START_Y + cursorTargetRow * MENU_ITEM_HEIGHT - 10, SCREEN_WIDTH, MENU_ITEM_HEIGHT);
            display.setDrawColor(0);
        }
        
        display.setCursor(8, y);
        display.print(currentMenu[itemIndex].label);
        
        // 显示通道模式
        display.setCursor(SCREEN_WIDTH - 40, y);
        switch (CHANNEL_MODE[itemIndex]) {
            case CHANNEL_MODE_OFFLINE:
                display.print("离线");
                break;
            case CHANNEL_MODE_ONLINE:
                display.print("在线");
                break;
            case CHANNEL_MODE_AUTO:
                // 自动模式下显示实际状态
                if (is_filament_online(itemIndex)) {
                    display.print("自动○");
                } else {
                    display.print("自动●");
                }
                break;
            default:
                display.print("未知");
                break;
        }
        
        if (isSelected) {
            display.setDrawColor(1);
        }
    }
    
    // 绘制滚动条
    if (menuItemCount > MENU_VISIBLE_ITEMS) {
        int contentHeight = MENU_VISIBLE_ITEMS * MENU_ITEM_HEIGHT;
        int scrollBarHeight = contentHeight * MENU_VISIBLE_ITEMS / menuItemCount;
        int scrollBarY = MENU_START_Y - 10 + (menuVisibleOffset * contentHeight / menuItemCount);
        display.drawFrame(SCREEN_WIDTH - 4, MENU_START_Y - 10, 4, contentHeight);
        display.drawBox(SCREEN_WIDTH - 3, scrollBarY, 2, scrollBarHeight);
    }
    
    // 绘制底部提示
    display.setFont(u8g2_font_wqy12_t_gb2312);
    display.setCursor(0, 62);
    display.print("<上    确认切换   下>");
    
    display.sendBuffer();
    last_drawn_state = SETUP_CHANNEL_STATUS;
}

// ============================
// 新增：关于页面
// ============================
void draw_setup_about() {
    display.clearBuffer();
    display.setFont(u8g2_font_wqy12_t_gb2312);
    
    // 标题
    const char* title = "关于DMS天枢V8";
    int titleX = (SCREEN_WIDTH - display.getStrWidth(title)) / 3;
    display.setCursor(titleX, 10);
    display.print(title);
    
    // 显示当前页内容
    int start_line = about_page * about_lines_per_page;
    for (int i = 0; i < about_lines_per_page; i++) {
        int line_index = start_line + i;
        if (line_index < total_about_lines) {
            display.setCursor(5, 23 + i * 12);
            display.print(about_info[line_index]);
        }
    }
    
    // 底部提示
    display.setCursor(0, 62);
    display.print("<       确认        V");
    
    display.sendBuffer();
    last_drawn_state = SETUP_ABOUT;
}

// ============================
// 按键处理（已更新）
// ============================
void handle_key_input() {
    if (screen_sleep) {
        if (digitalRead(KEY_FORWARD) == LOW || 
            digitalRead(KEY_CONFIRM) == LOW || 
            digitalRead(KEY_BACKWARD) == LOW) {
            screen_sleep = false;
            display.setPowerSave(0);
            last_action_time = millis();
            last_key_time = millis();
            delay(50);
        }
        return;
    }

    // 检测左键长按
    if (digitalRead(KEY_BACKWARD) == LOW) {
        if (!key_backward_pressed) {
            key_backward_pressed = true;
            key_backward_press_time = millis();
        }
    } else {
        if (key_backward_pressed) {
            key_backward_pressed = false;
            unsigned long press_duration = millis() - key_backward_press_time;
            
            if (press_duration < LONG_PRESS_DELAY) {
                // 短按：切换到上个选项
                switch(setup_state) {
                    case SETUP_NONE:
                        break;
                    case SETUP_MAIN:
                        menuPrev();
                        setup_selected = menuSelectedIndex;
                        break;
                    case SETUP_MOTOR_SELECT:
                        menuPrev();
                        motor_select_index = menuSelectedIndex;
                        break;
                    case SETUP_PULLBACK:
                        if (PULLBACK_TIMEOUT[motor_select_index] > 3) {
                            PULLBACK_TIMEOUT[motor_select_index]--;
                        }
                        break;
                    case SETUP_SLEEP:
                        if (SCREEN_ALWAYS_ON) {
                            SCREEN_ALWAYS_ON = false;
                            SLEEP_TIMEOUT_MIN = 15;
                        } else {
                            if (SLEEP_TIMEOUT_MIN > 1) {
                                SLEEP_TIMEOUT_MIN--;
                            }
                        }
                        break;
                    case SETUP_AMS_COUNT:  // AMS数量设置
                        if (AMS_COUNT > 1) {
                            AMS_COUNT--;
                        }
                        break;
                    case SETUP_DEBUG_MOTOR_SELECT:  // 电机选择菜单
                        menuPrev();
                        debug_selected_motor = menuSelectedIndex;
                        break;
                    case SETUP_DEBUG_CONTROL:  // 切换选中电机
                        debug_selected_motor--;
                        if (debug_selected_motor < 0) debug_selected_motor = 15;
                        break;
                    case SETUP_CHANNEL_STATUS:  // 通道选择菜单
                        menuPrev();
                        channel_status_selected = menuSelectedIndex;
                        break;
                    case SETUP_ABOUT:
                        // 关于页面无操作
                        break;
                }
            } else {
                // 长按：返回上级菜单
                switch(setup_state) {
                    case SETUP_NONE:
                        break;
                    case SETUP_MAIN:
                        setup_state = SETUP_NONE;
                        break;
                    case SETUP_MOTOR_SELECT:
                        setup_state = SETUP_MAIN;
                        break;
                    case SETUP_PULLBACK:
                        setup_state = SETUP_MOTOR_SELECT;
                        break;
                    case SETUP_SLEEP:
                        setup_state = SETUP_MAIN;
                        break;
                    case SETUP_DEBUG_MOTOR_SELECT:  // 返回主菜单
                        setup_state = SETUP_MAIN;
                        break;
                    case SETUP_DEBUG_CONTROL:  // 退出调试模式，停止所有电机
                        // 退出前停止所有电机
                        for (int i = 0; i < 16; i++) {
                            debug_motor_state[i] = 0;
                            motorCmd(i, 0);
                        }
                        setup_state = SETUP_DEBUG_MOTOR_SELECT;
                        break;
                    case SETUP_CHANNEL_STATUS:  // 返回主菜单
                        setup_state = SETUP_MAIN;
                        break;
                    case SETUP_AMS_COUNT:  // 返回主菜单
                        setup_state = SETUP_MAIN;
                        break;
                    case SETUP_ABOUT:
                        setup_state = SETUP_MAIN;
                        break;
                }
            }
            last_key_time = millis();
        }
    }

    if (key_pressed(KEY_FORWARD)) {
        switch(setup_state) {
            case SETUP_NONE:
                break;
            case SETUP_MAIN:
                menuNext();
                setup_selected = menuSelectedIndex;
                break;
            case SETUP_MOTOR_SELECT:
                menuNext();
                motor_select_index = menuSelectedIndex;
                break;
            case SETUP_PULLBACK:
                if (PULLBACK_TIMEOUT[motor_select_index] < 30) {
                    PULLBACK_TIMEOUT[motor_select_index]++;
                }
                break;
            case SETUP_SLEEP:
                if (SCREEN_ALWAYS_ON) {
                    SCREEN_ALWAYS_ON = false;
                    SLEEP_TIMEOUT_MIN = 15;
                } else {
                    SLEEP_TIMEOUT_MIN++;
                    if (SLEEP_TIMEOUT_MIN > 15) {
                        SCREEN_ALWAYS_ON = true;
                    }
                }
                break;
            case SETUP_DEBUG_MOTOR_SELECT:  // 电机选择菜单
                menuNext();
                debug_selected_motor = menuSelectedIndex;
                break;
            case SETUP_DEBUG_CONTROL:  // 切换选中电机
                debug_selected_motor++;
                if (debug_selected_motor >= 16) debug_selected_motor = 0;
                break;
            case SETUP_CHANNEL_STATUS:  // 通道选择菜单
                menuNext();
                channel_status_selected = menuSelectedIndex;
                break;
            case SETUP_AMS_COUNT:  // AMS数量设置
                if (AMS_COUNT < 4) {
                    AMS_COUNT++;
                }
                break;
            case SETUP_ABOUT:
                // 关于页面：确认键翻页
                const int total_pages = (total_about_lines + about_lines_per_page - 1) / about_lines_per_page;
                about_page = (about_page + 1) % total_pages;
                break;
        }
    }

    if (key_pressed(KEY_CONFIRM)) {
        switch(setup_state) {
            case SETUP_NONE:
                setup_state = SETUP_MAIN;
                setup_selected = 0;
                break;
            case SETUP_MAIN:
                if (setup_selected == 0) {
                    setup_state = SETUP_MOTOR_SELECT;
                    motor_select_index = 0;
                } else if (setup_selected == 1) {
                    setup_state = SETUP_SLEEP;
                } else if (setup_selected == 2) {  // 电机调试
                    setup_state = SETUP_DEBUG_MOTOR_SELECT;
                    debug_control_initialized = false;  // 强制重新初始化
                } else if (setup_selected == 3) {  // 通道状态设置（新增）
                    setup_state = SETUP_CHANNEL_STATUS;
                    channel_status_initialized = false;  // 强制重新初始化
                } else if (setup_selected == 4) {  // AMS状态设置（新增）
                    setup_state = SETUP_AMS_COUNT;
                } else if (setup_selected == 5) {  // 关于
                    setup_state = SETUP_ABOUT;
                }
                break;
            case SETUP_MOTOR_SELECT:
                setup_state = SETUP_PULLBACK;
                break;
            case SETUP_PULLBACK:
                setup_state = SETUP_MOTOR_SELECT;
                settings_save_to_flash();
                break;
            case SETUP_SLEEP:
                setup_state = SETUP_MAIN;
                settings_save_to_flash();
                break;
            case SETUP_DEBUG_MOTOR_SELECT:  // 进入电机调试控制界面
                setup_state = SETUP_DEBUG_CONTROL;
                break;
            case SETUP_DEBUG_CONTROL:  // 循环切换 停止->进料->退料->停止
                {
                    int current = debug_motor_state[debug_selected_motor];
                    int next;
                    
                    // 循环：停止(0) -> 进料(1) -> 退料(-1) -> 停止(0)
                    if (current == 0) {
                        next = 1;  // 停止->进料
                    } else if (current == 1) {
                        next = -1; // 进料->退料
                    } else {
                        next = 0;  // 退料->停止
                    }
                    
                    // 先停止其他电机（安全考虑，只允许一个电机动作）
                    for (int i = 0; i < 16; i++) {
                        if (i != debug_selected_motor && debug_motor_state[i] != 0) {
                            debug_motor_state[i] = 0;
                            motorCmd(i, 0);
                        }
                    }
                    
                    // 设置选中电机
                    debug_motor_state[debug_selected_motor] = next;
                    motorCmd(debug_selected_motor, next);
                }
                break;
            case SETUP_CHANNEL_STATUS:  // 切换通道模式（离线→在线→自动→离线）
                {
                    // 循环切换模式
                    CHANNEL_MODE[channel_status_selected] = (CHANNEL_MODE[channel_status_selected] + 1) % 3;
                    
                    // 根据新模式更新通道状态
                    if (CHANNEL_MODE[channel_status_selected] == CHANNEL_MODE_OFFLINE) {
                        set_filament_online(channel_status_selected / 4, channel_status_selected % 4, false);
                    } else if (CHANNEL_MODE[channel_status_selected] == CHANNEL_MODE_ONLINE) {
                        set_filament_online(channel_status_selected / 4, channel_status_selected % 4, true);
                    } else if (CHANNEL_MODE[channel_status_selected] == CHANNEL_MODE_AUTO) {
                        // 自动模式：根据微动开关状态设置
                        if (channel_status_selected < 8) {  // 只处理1-8通道
                            int pin = MICROSWITCH_PINS[channel_status_selected];
                            bool switchState = digitalRead(pin);
                            set_filament_online(channel_status_selected / 4, channel_status_selected % 4, switchState);
                        }
                    }
                    
                    settings_save_to_flash();  // 保存设置到Flash
                }
                break;
            case SETUP_AMS_COUNT:  // 保存AMS数量设置
                setup_state = SETUP_MAIN;
                settings_save_to_flash();
                break;
            case SETUP_ABOUT:
                setup_state = SETUP_MAIN;
                break;
        }
    }
}

void ssd1306_update_extruder_data() {
    if (screen_sleep) {
        handle_key_input();
        return;
    }

    handle_key_input();

    if (setup_state != SETUP_NONE) {
        return;
    }

    for (int i=0; i<16; i++) {
        display_data.extruder_online[i] = is_filament_online(i);
        display_data.extruder_length[i] = get_filament_meters(i);

        _filament_motion_state_set motion = get_filament_motion(i);
        
        bool currently_feeding = (motion == need_send_out || motion == act_send_mm) ||
                                 (display_data.extruder_state[i][0] == '<');
        bool currently_pulling = (is_need_pull_back && now_filament_num_need_pull_back == i) ||
                                 (display_data.extruder_state[i][0] == '>');
        
        if (currently_feeding && !prev_extruder_state_feeding[i]) {
            in_feeding_process[i] = true;
        }
        if (!currently_feeding && prev_extruder_state_feeding[i] && in_feeding_process[i]) {
            in_feeding_process[i] = false;
            action_complete_time[i] = millis();
        }
        if (currently_pulling && !prev_extruder_state_pulling[i]) {
            in_pulling_process[i] = true;
        }
        if (!currently_pulling && prev_extruder_state_pulling[i] && in_pulling_process[i]) {
            in_pulling_process[i] = false;
            action_complete_time[i] = millis();
        }
        
        prev_extruder_state_feeding[i] = currently_feeding;
        prev_extruder_state_pulling[i] = currently_pulling;

        if (motion == need_send_out || motion == act_send_mm) {
            display_data.extruder_state[i][0] = '<';
        } else if (motion == need_pull_back || motion == act_pull_mm) {
            if (!is_need_pull_back || (now_filament_num_need_pull_back != i)) {
                display_data.extruder_state[i][0] = '-';
            } else {
                display_data.extruder_state[i][0] = '>';
            }
        } else {
            display_data.extruder_state[i][0] = '-';
        }
        display_data.extruder_state[i][1] = '\0';
    }

    display_data.current_slot = get_now_filament_num();
    display_data.ams_auto_mode = ams_enable;
    display_data.as5600_ok = enable_as5600 && if_as5600_init;
    display_data.motor_running = execute_motion;

    bool any_in_process = false;
    for (int i = 0; i < 16; i++) {
        if (in_feeding_process[i] || in_pulling_process[i]) {
            any_in_process = true;
            break;
        }
        if (millis() - action_complete_time[i] < ACTION_COMPLETE_DELAY) {
            any_in_process = true;
            break;
        }
    }
    
    displaymode = !any_in_process;
    update_screen_sleep();
}

void drawMainScreen() {
    if (displaymode) {
        display.clearBuffer();
        display.setFont(u8g2_font_logisoso62_tn);
        
        int show_num = display_data.current_slot + 1;
        for (int i=0; i<16; i++) {
            if (display_data.extruder_state[i][0] == '<') {
                show_num = i + 1;
                break;
            }
        }

        char buf[4];
        sprintf(buf, "%02d", show_num);
        display.setCursor(20, 62);
        display.print(buf);
        display.sendBuffer();
    } else {
        display.clearBuffer();
        display.setFont(u8g2_font_wqy12_t_gb2312);

        display.setCursor(0, 13);
        display.print("模式:");
        if (display_data.ams_auto_mode) {
            display.print("自动");
        } else {
            display.print("手动");
        }
        
        int current_motor = display_data.current_slot;
        if (current_motor >= 0 && current_motor < 16) {
            display.setCursor(64, 13);
            display.print("退料:");
            display.print(PULLBACK_TIMEOUT[current_motor]);
            display.print("秒");
        }

        display.setCursor(0, 26);
        display.print("电机:");
        if (display_data.motor_running) {
            display.print("运行");
        } else {
            display.print("停止");
        }
        display.setCursor(64, 26);
        display.print("通道:");
        display.print(display_data.current_slot + 1);

        int remain_seconds = 0;
        if (is_need_pull_back && now_filament_num_need_pull_back >= 0 && now_filament_num_need_pull_back < 16) {
            unsigned long elapsed = millis() - last_pullback_time;
            int timeout_ms = PULLBACK_TIMEOUT[now_filament_num_need_pull_back] * 1000;
            remain_seconds = (timeout_ms - elapsed) / 1000;
            if (remain_seconds < 0) remain_seconds = 0;
        }

        // 只显示正在进料/退料的通道
        int active_channel = -1;
        for (int i = 0; i < 16; i++) {
            if (display_data.extruder_state[i][0] == '<' || display_data.extruder_state[i][0] == '>') {
                active_channel = i;
                break;
            }
        }

        if (active_channel >= 0) {
            int y = 42;
            display.setCursor(0, y);
            display.print("通道");
            display.print(active_channel + 1);
            display.print(":");
            const char* state = get_state_word(display_data.extruder_state[active_channel][0]);
            display.print(state);
            
            if (!display_data.extruder_online[active_channel]) {
                display.print(" 离线");
            } else if (active_channel == now_filament_num_need_pull_back && is_need_pull_back) {
                display.print(" 剩");
                display.print(remain_seconds);
                display.print("秒");
            }
        }

        display.sendBuffer();
    }
    
    last_drawn_state = SETUP_NONE;
}

void ssd1306_refresh_screen() {
    if (screen_sleep) {
        return;
    }

    if (setup_state != SETUP_NONE) {
        switch(setup_state) {
            case SETUP_MAIN: draw_setup_main(); break;
            case SETUP_MOTOR_SELECT: draw_motor_select(); break;
            case SETUP_PULLBACK: draw_setup_pullback(); break;
            case SETUP_SLEEP: draw_setup_sleep(); break;
            case SETUP_DEBUG_MOTOR_SELECT: draw_debug_motor_select(); break;
            case SETUP_DEBUG_CONTROL: draw_debug_control(); break;
            case SETUP_CHANNEL_STATUS: draw_channel_status(); break;  // 新增
            case SETUP_AMS_COUNT: draw_setup_ams_count(); break;  // 新增
            case SETUP_ABOUT: draw_setup_about(); break;
        }
    } else {
        drawMainScreen();
    }
}