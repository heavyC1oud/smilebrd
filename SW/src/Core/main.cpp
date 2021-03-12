
#include "scmRTOS.h"
#include "board.h"


int main()
{
    initMCU();

    pin_led.On();
    delayMs(150);
    pin_led.Off();

    // run tasks
    OS::run();
}
