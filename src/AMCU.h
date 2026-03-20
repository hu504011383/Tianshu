#pragma once

#include "main.h"
#include "BambuBus.h"
#ifdef __cplusplus
extern "C"
{
#endif

    // #define AMCU_uart uart1
    // #define AMCU_uart_IRQ UART1_IRQ
    // #define AMCU_pin_tx 4
    // #define AMCU_pin_rx 5

#define AMCU_AS5600_SDA 4
#define AMCU_AS5600_SCL 5

// AS5600相关常量
#define AS5600_RAW_TO_DEGREES     360.0 / 4096
#define AS5600_DEGREES_TO_RAW     4096 / 360.0
#define AS5600_PI 3.14159265358979323846
#define AS5600_MM_PER_CNT         -(AS5600_PI * 7.5f) / 4096.0f

// AS5600磁场状态
enum AS5600_magnet_stu {
    AS5600_low     = 1,
    AS5600_high    = 2,
    AS5600_offline = -1,
    AS5600_normal  = 0
};

// AS5600模拟函数
extern void AS5600_init();
extern float AS5600_get_distance_E();
extern uint16_t AS5600_get_raw_angle(uint8_t channel);
extern AS5600_magnet_stu AS5600_get_magnet_stu(uint8_t channel);
extern bool AS5600_is_online(uint8_t channel);
extern float AS5600_get_speed(uint8_t channel);

// AS5600模拟变量
extern int16_t as5600_current_angle[4];
extern int16_t as5600_last_angle[4];
extern uint32_t as5600_last_time[4];
extern float as5600_speed[4];

    void set_color(u_int8_t r, u_int8_t g, u_int8_t b, u_int8_t a);
    void rgb_set_breath(int16_t time, uint16_t count);
    void selectOnePos(char num);
    void releaseAllPos();
    void set_LED_state(char led, char state);
    void debug_info(bool force);

    extern void AMCU_init();
    extern void AMCU_run();
    extern void AMCU_motion();

    void motorCmd(int motorNum, int cmd);
    void setMainMotorDutyCycle(uint8_t dutyCycle);
    void init_io_pins();

#ifdef __cplusplus
}
#endif
