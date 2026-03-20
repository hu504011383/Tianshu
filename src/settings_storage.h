#pragma once
#include <Arduino.h>

// ============================
// 设置存储管理器 - 使用RP2040 Flash直接存储
// 兼容Arduino mbed核心 (framework-arduino-mbed)
// ============================

// Flash存储配置 - 使用与BambuBus不同的地址区域
// RP2040 Flash大小通常为2MB，程序使用前面部分，我们在靠后位置存储设置
#define SETTINGS_FLASH_OFFSET       (1024 * 1024)  // 1MB偏移处，远离程序代码
#define SETTINGS_FLASH_SIZE         4096            // 一个扇区大小

// 通道状态模式定义
typedef enum {
    CHANNEL_MODE_OFFLINE = 0,    // 离线
    CHANNEL_MODE_ONLINE = 1,     // 在线
    CHANNEL_MODE_AUTO = 2         // 自动（根据微动开关）
} ChannelMode;

// 设置数据结构 - 支持16个电机独立的退料时间
struct SettingsData {
    int32_t pullback_timeout[16];    // 16个电机的退料时间（秒）
    int32_t sleep_timeout_min;      // 息屏时间（分钟）
    int32_t screen_always_on;       // 常亮标记（0或1）
    int32_t channel_mode[16];       // 16个通道的状态模式（0=离线, 1=在线, 2=自动）
    int32_t ams_count;             // AMS数量（1-4）
    uint32_t checksum;              // 校验和
    uint32_t magic;                 // 魔数
};

// 魔数定义 - 版本5（结构体改变，更新魔数）
#define SETTINGS_MAGIC_NUMBER    0x53415425  // "STA%" = Settings Active V5

// 默认值定义
#define DEFAULT_PULLBACK_TIMEOUT    7
#define DEFAULT_SLEEP_TIMEOUT_MIN   15
#define DEFAULT_SCREEN_ALWAYS_ON    0
#define DEFAULT_AMS_COUNT         4

// 初始化设置存储（在setup中调用）
void settings_storage_init();

// 从Flash加载设置到变量
void settings_load_from_flash();

// 保存当前设置到Flash
void settings_save_to_flash();

// 重置为默认值（不自动保存，需手动调用save）
void settings_reset_to_defaults();

// 检查设置是否有效（校验和验证）
bool settings_is_valid();

// 获取指定电机的退料超时时间（新增辅助函数）
int get_motor_pullback_timeout(int motor_index);