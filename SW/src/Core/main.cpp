
#include "board.h"


int main()
{
    initMCU();

    while(1) {
        handleUARTData();

        pin_led.Cpl();
        pin_pc6.Cpl();
        pin_pc7.Cpl();
        pin_pc8.Cpl();
        pin_pc9.Cpl();
        delayMs(200);
    }
}
