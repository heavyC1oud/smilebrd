#ifndef _BOARD_H_
#define _BOARD_H_

#include <cstdint>
#include "stm32l475_pin_cpp.h"
#include "uart1.h"


typedef Pin<'A', 5, 'H', PIN_SPEED_HIGH> pin_led_t;
typedef Pin<'A', 9, 'H', PIN_SPEED_HIGH> pin_uart1_tx_t;
typedef Pin<'A', 10, 'H', PIN_SPEED_HIGH> pin_uart1_rx_t;

typedef Pin<'B', 12, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io1_sample_t;
typedef Pin<'B', 13, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io2_sense_t;
typedef Pin<'B', 14, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io3_sense_t;
typedef Pin<'B', 15, 'H', PIN_SPEED_HIGH> pin_tsc_g1_io4_sense_t;

typedef Pin<'C', 6, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io1_sample_t;
typedef Pin<'C', 7, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io2_sense_t;
typedef Pin<'C', 8, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io3_sense_t;
typedef Pin<'C', 9, 'H', PIN_SPEED_HIGH> pin_tsc_g4_io4_sense_t;

extern pin_uart1_tx_t pin_uart1_tx;
extern pin_uart1_rx_t pin_uart1_rx;

extern pin_led_t pin_led;

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
void handleSensors();
uint64_t getSysTick();
void delayMs(uint64_t delay);



#endif  // _BOARD_H_
