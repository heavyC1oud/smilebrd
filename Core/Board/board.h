#ifndef _BOARD_H_
#define _BOARD_H_

#include <cstdint>
#include "stm32l475_pin_cpp.h"
#include "uart1.h"


typedef Pin<'A', 2, 'H', PIN_SPEED_HIGH> pin_led_but_4_t;
typedef Pin<'A', 4, 'H', PIN_SPEED_HIGH> pin_led_but_6_t;
typedef Pin<'A', 5, 'H', PIN_SPEED_HIGH> pin_led_but_1_t;
typedef Pin<'A', 6, 'H', PIN_SPEED_HIGH> pin_led_but_2_t;
typedef Pin<'A', 7, 'H', PIN_SPEED_HIGH> pin_led_but_3_t;

typedef Pin<'A', 9, 'H', PIN_SPEED_HIGH> pin_uart1_tx_t;
typedef Pin<'A', 10, 'H', PIN_SPEED_HIGH> pin_uart1_rx_t;
typedef Pin<'A', 15, 'H', PIN_SPEED_HIGH> pin_led_but_5_t;

typedef Pin<'B', 3, 'H', PIN_SPEED_HIGH> pin_out_i2c_led_green_t;
typedef Pin<'B', 4, 'H', PIN_SPEED_HIGH> pin_out_power_led_green_t;
typedef Pin<'B', 5, 'H', PIN_SPEED_HIGH> pin_out_power_led_red_t;
typedef Pin<'B', 3, 'H', PIN_SPEED_HIGH> pin_out_i2c_led_red_t;

typedef Pin<'B', 7, 'L', PIN_SPEED_HIGH> pin_touch_sig_t;
typedef Pin<'B', 8, 'H', PIN_SPEED_HIGH> pin_i2c1_scl_t;
typedef Pin<'B', 9, 'H', PIN_SPEED_HIGH> pin_i2c1_sda_t;
typedef Pin<'B', 12, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io1_sample_t;
typedef Pin<'B', 13, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io2_sense_t;
typedef Pin<'B', 14, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io3_sense_t;
typedef Pin<'B', 15, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io4_sense_t;

typedef Pin<'C', 6, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io1_sample_t;
typedef Pin<'C', 7, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io2_sense_t;
typedef Pin<'C', 8, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io3_sense_t;
typedef Pin<'C', 9, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io4_sense_t;
typedef Pin<'C', 11, 'H', PIN_SPEED_HIGH> pin_out_server_led_green_t;
typedef Pin<'C', 12, 'H', PIN_SPEED_HIGH> pin_out_server_led_red_t;
typedef Pin<'C', 13, 'H', PIN_SPEED_HIGH> pin_in_server_led_t;

extern pin_uart1_tx_t pin_uart1_tx;
extern pin_uart1_rx_t pin_uart1_rx;

extern pin_i2c1_scl_t pin_i2c1_scl;
extern pin_i2c1_sda_t pin_i2c1_sda;

extern pin_led_but_1_t pin_led_but_1;
extern pin_led_but_2_t pin_led_but_2;
extern pin_led_but_3_t pin_led_but_3;
extern pin_led_but_4_t pin_led_but_4;
extern pin_led_but_5_t pin_led_but_5;
extern pin_led_but_6_t pin_led_but_6;

extern pin_in_server_led_t pin_in_server_led;
extern pin_out_server_led_green_t pin_out_server_led_green;
extern pin_out_server_led_red_t pin_out_server_led_red;

extern pin_out_i2c_led_green_t pin_out_i2c_led_green;
extern pin_out_i2c_led_red_t pin_out_i2c_led_red;

extern pin_out_power_led_green_t pin_out_power_led_green;
extern pin_out_power_led_red_t pin_out_power_led_red;

extern pin_touch_sig_t pin_touch_sig;

extern pin_tsc_g1_io1_sample_t pin_tsc_g1_io1_sample;
extern pin_tsc_g1_io2_sense_t pin_tsc_g1_io2_sense;
extern pin_tsc_g1_io3_sense_t pin_tsc_g1_io3_sense;
extern pin_tsc_g1_io4_sense_t pin_tsc_g1_io4_sense;

extern pin_tsc_g4_io1_sample_t pin_tsc_g4_io1_sample;
extern pin_tsc_g4_io2_sense_t pin_tsc_g4_io2_sense;
extern pin_tsc_g4_io3_sense_t pin_tsc_g4_io3_sense;
extern pin_tsc_g4_io4_sense_t pin_tsc_g4_io4_sense;


void initMCU();
void handleUARTData();

uint64_t getSysTick();
void delayMs(uint64_t delay);

#endif  // _BOARD_H_
