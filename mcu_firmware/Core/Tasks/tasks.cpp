
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

            sleep(1);
        }
    }

    /**
     * @brief GPIO handling
     */
    template<>
    OS_PROCESS void ThandleGPIO::exec() {
        while(1) {
            const uint8_t sensors = keyb.getSensors();

            // set touch signal to BBB
            if(sensors) {
                pin_touch_sig.On();
                // pin_out_power_led_red.On();
            }
            else {
                pin_touch_sig.Off();
                // pin_out_power_led_red.Off();
            }

            // blink button led
            keyb.blinkLed(KEYB_BUT_1);
            keyb.blinkLed(KEYB_BUT_2);
            keyb.blinkLed(KEYB_BUT_3);
            keyb.blinkLed(KEYB_BUT_4);
            keyb.blinkLed(KEYB_BUT_5);
            keyb.blinkLed(KEYB_BUT_6);

            //blink left eye led
            keyb.blinkLeftLed();

            // server enable signal
            if(pin_in_server_led.Signalled()) {
                pin_out_server_led_green.On();
                pin_out_server_led_red.Off();
            }
            else {
                pin_out_server_led_green.Off();
                pin_out_server_led_red.On();
            }

            sleep(1);
        }
    }
}

ThandleTSC handleTSC;
ThandleGPIO handleGPIO;
