#include "settings_storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// 外部变量声明（来自ssd1306_display.cpp）
extern int PULLBACK_TIMEOUT[16];  // 现在是数组
extern int SLEEP_TIMEOUT_MIN;
extern bool SCREEN_ALWAYS_ON;
extern int CHANNEL_MODE[16];  // 通道状态模式数组
extern int AMS_COUNT;  // AMS数量

// 计算校验和
static uint32_t calculate_checksum(const SettingsData* data) {
    uint32_t checksum = 0;
    for (int i = 0; i < 16; i++) {
        checksum += (uint32_t)data->pullback_timeout[i] * (i + 1);
        checksum += (uint32_t)data->channel_mode[i] * (i + 17);
    }
    checksum += (uint32_t)data->sleep_timeout_min * 100;
    checksum += (uint32_t)data->screen_always_on * 10000;
    checksum += (uint32_t)data->ams_count * 1000;
    return checksum;
}

// 从Flash读取设置数据
static void read_settings_from_flash(SettingsData* data) {
    // 获取Flash基地址
    const uint8_t* flash_base = (const uint8_t*)XIP_BASE;
    // 计算设置存储地址
    const uint8_t* settings_addr = flash_base + SETTINGS_FLASH_OFFSET;
    
    // 复制数据
    memcpy(data, settings_addr, sizeof(SettingsData));
}

// 写入设置到Flash（需要禁用中断，擦除后写入）
static void write_settings_to_flash(const SettingsData* data) {
    // 准备缓冲区（必须是Flash页大小的倍数，256字节对齐）
    static uint8_t buffer[FLASH_SECTOR_SIZE] __attribute__((aligned(256)));
    
    // 清空缓冲区
    memset(buffer, 0xFF, sizeof(buffer));
    
    // 复制设置数据到缓冲区开头
    memcpy(buffer, data, sizeof(SettingsData));
    
    // 禁用中断并保存状态
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    // 擦除扇区
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    
    // 写入数据
    flash_range_program(SETTINGS_FLASH_OFFSET, buffer, FLASH_SECTOR_SIZE);
    
    // 恢复中断状态
    __set_PRIMASK(primask);
}

void settings_storage_init() {
    SettingsData data;
    
    // 从Flash读取
    read_settings_from_flash(&data);
    
    // 检查魔数
    if (data.magic != SETTINGS_MAGIC_NUMBER) {
        // 首次使用或数据损坏，使用默认值
        settings_reset_to_defaults();
        settings_save_to_flash();
        return;
    }
    
    // 验证校验和
    uint32_t calc_checksum = calculate_checksum(&data);
    if (calc_checksum != data.checksum) {
        // 校验失败，使用默认值
        settings_reset_to_defaults();
        settings_save_to_flash();
        return;
    }
    
    // 校验通过，加载值
    // 范围检查
    for (int i = 0; i < 16; i++) {
        if (data.pullback_timeout[i] >= 3 && data.pullback_timeout[i] <= 30) {
            PULLBACK_TIMEOUT[i] = data.pullback_timeout[i];
        } else {
            PULLBACK_TIMEOUT[i] = DEFAULT_PULLBACK_TIMEOUT;
        }
        
        if (data.channel_mode[i] >= 0 && data.channel_mode[i] <= 2) {
            CHANNEL_MODE[i] = data.channel_mode[i];
        } else {
            CHANNEL_MODE[i] = CHANNEL_MODE_OFFLINE;
        }
    }
    
    if (data.sleep_timeout_min >= 1 && data.sleep_timeout_min <= 16) {
        SLEEP_TIMEOUT_MIN = data.sleep_timeout_min;
    } else {
        SLEEP_TIMEOUT_MIN = DEFAULT_SLEEP_TIMEOUT_MIN;
    }
    
    SCREEN_ALWAYS_ON = (data.screen_always_on != 0);
    
    if (data.ams_count >= 1 && data.ams_count <= 4) {
        AMS_COUNT = data.ams_count;
    } else {
        AMS_COUNT = DEFAULT_AMS_COUNT;
    }
}

void settings_load_from_flash() {
    // 重新加载（与init相同逻辑）
    settings_storage_init();
}

void settings_save_to_flash() {
    SettingsData data;
    
    // 填充数据
    for (int i = 0; i < 16; i++) {
        data.pullback_timeout[i] = PULLBACK_TIMEOUT[i];
        data.channel_mode[i] = CHANNEL_MODE[i];
    }
    data.sleep_timeout_min = SLEEP_TIMEOUT_MIN;
    data.screen_always_on = SCREEN_ALWAYS_ON ? 1 : 0;
    data.ams_count = AMS_COUNT;
    data.checksum = calculate_checksum(&data);
    data.magic = SETTINGS_MAGIC_NUMBER;
    
    // 写入Flash
    write_settings_to_flash(&data);
}

void settings_reset_to_defaults() {
    for (int i = 0; i < 16; i++) {
        PULLBACK_TIMEOUT[i] = DEFAULT_PULLBACK_TIMEOUT;
        CHANNEL_MODE[i] = CHANNEL_MODE_OFFLINE;
    }
    SLEEP_TIMEOUT_MIN = DEFAULT_SLEEP_TIMEOUT_MIN;
    SCREEN_ALWAYS_ON = DEFAULT_SCREEN_ALWAYS_ON;
    AMS_COUNT = DEFAULT_AMS_COUNT;
}

bool settings_is_valid() {
    SettingsData data;
    read_settings_from_flash(&data);
    
    if (data.magic != SETTINGS_MAGIC_NUMBER) {
        return false;
    }
    
    uint32_t calc_checksum = calculate_checksum(&data);
    return (calc_checksum == data.checksum);
}

// 获取指定电机的退料超时时间
int get_motor_pullback_timeout(int motor_index) {
    if (motor_index >= 0 && motor_index < 16) {
        return PULLBACK_TIMEOUT[motor_index];
    }
    return DEFAULT_PULLBACK_TIMEOUT;
}