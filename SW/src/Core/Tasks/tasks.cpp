
#include "board.h"
#include "uart1.h"
#include "i2c1.h"
#include "keyboard.h"

#include "tasks.h"


namespace OS {
    /**
     * @brief Sensors handling
     */
    template<>
    OS_PROCESS void ThandleTSC::exec() {
        while(1) {
            keyb.handle();

            i2c1.setData(keyb.getSensors());

            if(keyb.getSensors()) {
                pin_led.On();
                // send signal to BBB
                pin_touch_sig.On();
            }
            else {
                pin_led.Off();
                // end send signal to BBB
                pin_touch_sig.Off();
            }
        }
    }
}

ThandleTSC handleTSC;
