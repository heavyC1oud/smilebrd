#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "stm32l475_tsc_cpp.h"

typedef enum
{
    KEYB_SENSING_LIMIT = 300,       // defference of sampling values for touch detection
} keyb_param_t;

typedef enum
{
    KEYB_BUT_1,
    KEYB_BUT_2,
    KEYB_BUT_3,
    KEYB_BUT_4,
    KEYB_BUT_5,
    KEYB_BUT_6,
    KEYB_BUT_COUNT,
} keyb_but_num_t;

typedef enum{
    KEYB_BUT_LED_HOLD_MS = 20,
    KEYB_POW_LED_HOLD_MS = 1,
} keyb_led_hold_t;


class keyb_t
{
private:
    uint32_t calibrate_value[6];
    uint8_t sensor_value;

private:
    void poll(tsc_io_no_t io, uint32_t* group1_val, uint32_t* group4_val);

public:
    void init();
    void calibrate();
    void handle();
    bool getSensor(keyb_but_num_t num);
    uint8_t getSensors();
    void blinkLed(keyb_but_num_t num);
    void blinkLeftLed();
};

extern keyb_t keyb;

#endif  // _KEYBOARD_H_
