
#include "board.h"


int main()
{
    initMCU();

    pin_led.On();
    delayMs(500);
    pin_led.Off();

    while(1) {
        handleUARTData();

        handleSensors();
    }
}
