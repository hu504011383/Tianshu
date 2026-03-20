#include "main.h"
#include "mbed.h"
#include "multicore.h"
#include "ssd1306_display.h"
#include "settings_storage.h"

// 新增：测试计数器（用于屏幕显示，可替换为你的业务数据）
int test_counter = 0;

// 新增：屏幕刷新间隔配置（100ms刷新一次，不影响业务逻辑）
#define SCREEN_REFRESH_MS 200

void watchdog_init(){
    watchdog_enable(10000, true);
}

void watchdog_feed(){
    watchdog_update();
}
void setup(){
    watchdog_init();
    
    // 初始化设置存储（在AMCU_init之前，确保变量先被加载）
    settings_storage_init(); 
    // ========== 新增：初始化显示屏（保留原有代码结构，仅追加） ==========
    Wire.begin();  // 初始化 I2C
    AMCU_init(); // This calls Serial.begin internally now
    ssd1306_init(); // 初始化屏幕
    
    

    // motorCmd(0, 1); // 原有注释代码保留
}

void loop(){
    // 新增：屏幕刷新计时变量（静态变量，仅初始化一次）
    static unsigned long last_display = 0;
    
    while (1) {
        // 原有核心逻辑：喂狗 → 串口调试 → AMCU业务
        watchdog_feed();
        AMCU_run();      // Handles BMCU protocol and BambuBus

        // 检查微动开关状态并更新自动模式下的通道状态
        check_microswitch_status();

        // ========== 新增：核心0定时刷新屏幕（不阻塞原有逻辑） ==========
         if (millis() - last_display >= SCREEN_REFRESH_MS) {
             ssd1306_update_extruder_data();
             ssd1306_refresh_screen();
             last_display = millis();
         }
    }
}
