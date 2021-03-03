#ifndef _BOARD_H_
#define _BOARD_H_

#include <cstdint>
#include "stm32l475_pin_cpp.h"
#include "uart1.h"


typedef Pin<'A', 5, 'H', PIN_SPEED_HIGH> pin_led_t;
typedef Pin<'A', 9, 'H', PIN_SPEED_HIGH> pin_uart1_tx_t;
typedef Pin<'A', 10, 'H', PIN_SPEED_HIGH> pin_uart1_rx_t;

typedef Pin<'B', 12, 'H', PIN_SPEED_HIGH> pin_pb12_t;
typedef Pin<'B', 13, 'H', PIN_SPEED_HIGH> pin_pb13_t;
typedef Pin<'B', 14, 'H', PIN_SPEED_HIGH> pin_pb14_t;
typedef Pin<'B', 15, 'H', PIN_SPEED_HIGH> pin_pb15_t;

typedef Pin<'C', 6, 'H', PIN_SPEED_HIGH> pin_pc6_t;
typedef Pin<'C', 7, 'H', PIN_SPEED_HIGH> pin_pc7_t;
typedef Pin<'C', 8, 'H', PIN_SPEED_HIGH> pin_pc8_t;
typedef Pin<'C', 9, 'H', PIN_SPEED_HIGH> pin_pc9_t;

extern pin_uart1_tx_t pin_uart1_tx;
extern pin_uart1_rx_t pin_uart1_rx;

extern pin_led_t pin_led;

extern pin_pb12_t pin_pb12;
extern pin_pb13_t pin_pb13;
extern pin_pb14_t pin_pb14;
extern pin_pb15_t pin_pb15;

extern pin_pc6_t pin_pc6;
extern pin_pc7_t pin_pc7;
extern pin_pc8_t pin_pc8;
extern pin_pc9_t pin_pc9;


void initMCU();
void handleUARTData();
uint64_t getSysTick();
void delayMs(uint64_t delay);



#endif  // _BOARD_H_
