#include <cstring>
#include "stm32l4xx.h"

#include "stm32l475_rcc_cpp.h"
#include "stm32l475_flash_cpp.h"
#include "stm32l475_tmr_cpp.h"
#include "dataflash.h"
#include "uart1.h"
#include "keyboard.h"
#include "i2c1.h"

#include "board.h"

static Tmr<TMR_15> delayTmr;


// delay counter, 1 tick = 1 ms
volatile static uint64_t delayTick = 0;

static void initClock();
static void initGPIO();
static void initUART();
static void initDataflash();
static void initKeys();
static void initI2C();


/**
 * @brief MCU initialization
 */
void initMCU()
{
    initGPIO();
    initClock();
    initDataflash();
    initUART();
    initKeys();
    initI2C();
}


/**
 * @brief GPIO initialization
 */
void initGPIO()
{
    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOA);
    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOB);
    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOC);

    pin_led.Mode(OUTPUT);
    pin_led.Off();

    pin_touch_sig.Mode(OUTPUT);
    pin_touch_sig.Off();
}

/**
 * @brief System clock initialization
 * system clock 80 MHz
 */
static void initClock()
{
    flash_t::prefetch_en();
    flash_t::latency(FLASH_LATENCY_4);
    while(flash_t::get_latency() != FLASH_LATENCY_4);

    rcc_t::deinit();

    rcc_t::hsi_on();
    while(rcc_t::is_hsi_ready() == 0);

    rcc_t::pll_config(RCC_PLL_HSI16, RCC_PLL_M_1, 10, RCC_PLL_P_7, RCC_PLL_Q_2, RCC_PLL_R_2);
    rcc_t::pll_r_on();
    rcc_t::pll_on();
    while(rcc_t::is_pll_ready() == 0);

    // PLL 80 MHz
    rcc_t::sysclock_source(RCC_SYSCLOCK_PLL);
    while(rcc_t::get_sysclock_source() != RCC_SYSCLOCK_PLL);

    rcc_t::ahbclock_div(RCC_AHBCLOCK_DIV1);
    rcc_t::apb1clock_div(RCC_APBCLOCK_DIV1);
    rcc_t::apb2clock_div(RCC_APBCLOCK_DIV1);

    SystemCoreClockUpdate();

    // init timer 15 as system delay timer
    delayTmr.init();
    // set 40 MHz clock
    delayTmr.set_freq(SystemCoreClock / 2);
    // set 1 ms overflow
    delayTmr.set_arr(SystemCoreClock / 2000);
    delayTmr.enable_irq();
    delayTmr.on();
}


/**
 * @brief UART initialization
 */
void initUART()
{
    uart.init();
}


/**
 * @brief Dataflash initialization
 */
static void initDataflash()
{
    dataflash_t::initDataflash();
}


/**
 * @brief Keyboard initialization
 */
void initKeys()
{
    keyb.init();

    keyb.calibrate();
}

/**
 * @brief Keyboard initialization
 */
void initI2C()
{
    i2c1.init();
}


/**
 * @brief UART data handling
 */
void handleUARTData()
{
    // if there is incoming data
    if(uart.getRxBufIndex() != 0) {
        // disable receiver
        uart.receiveDis();

        // loopback
        uart.sendData(uart.getBuf(), uart.getRxBufIndex());
        uart.setRxBufIndex(0);
    }
}

/**
 * @brief Get delay tick value
 */
uint64_t getDelayTick()
{
    return delayTick;
}


/**
 * @brief Set delay in milliseconds
 *        TIM15 ISR used
 *
 * @param delay - delay in milliseconds
 */
void delayMs(uint64_t delay)
{
    volatile uint64_t tickStart = getDelayTick();

    while((getDelayTick() - tickStart) < delay);
}

/**
 * @brief TIM15 ISR handler
 */
extern "C" void TIM1_BRK_TIM15_IRQHandler(void)
{
    delayTmr.set_status(~TIM_SR_UIF);

    delayTick++;
}
