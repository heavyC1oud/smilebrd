#include <cstring>
#include "stm32l4xx.h"

#include "stm32l475_rcc_cpp.h"
#include "stm32l475_flash_cpp.h"
#include "dataflash.h"
#include "uart1.h"

#include "board.h"


// sysTick counter, 1 tick = 1 ms
volatile static uint64_t sysTick = 0;

static void initClock();
static void initGPIO();
static void initUART();
static void initDataflash();



/**
 * @brief MCU initialization
 */
void initMCU()
{
    initGPIO();
    initClock();
    initDataflash();
    initUART();
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

    // pin_pb12.Mode(OUTPUT);
    // pin_pb12.Off();

    // pin_pb13.Mode(OUTPUT);
    // pin_pb13.Off();

    // pin_pb14.Mode(OUTPUT);
    // pin_pb14.Off();

    // pin_pb15.Mode(OUTPUT);
    // pin_pb15.Off();

    pin_pc6.Mode(OUTPUT);
    pin_pc6.Off();

    pin_pc7.Mode(OUTPUT);
    pin_pc7.Off();

    pin_pc8.Mode(OUTPUT);
    pin_pc8.Off();

    pin_pc9.Mode(OUTPUT);
    pin_pc9.Off();
}

/**
 * @brief System clock initialization
 * system clock 12 MHz
 */
static void initClock()
{
    flash_t::prefetch_en();
    flash_t::latency(FLASH_LATENCY_0);
    while(flash_t::get_latency() != FLASH_LATENCY_0);

    rcc_t::deinit();

    rcc_t::hse_on();
    while(rcc_t::is_hse_ready() == 0);

    // HSE 12 MHz
    rcc_t::sysclock_source(RCC_SYSCLOCK_HSE);
    while(rcc_t::get_sysclock_source() != RCC_SYSCLOCK_HSE);

    rcc_t::ahbclock_div(RCC_AHBCLOCK_DIV1);
    rcc_t::apb1clock_div(RCC_APBCLOCK_DIV1);
    rcc_t::apb2clock_div(RCC_APBCLOCK_DIV1);

    SystemCoreClockUpdate();

    // enable sysTick to 1 ms period, default interrupt priority = 3
    SysTick_Config(SystemCoreClock / 1000);
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
 * @brief Get system tick value
 */
uint64_t getSysTick()
{
    return sysTick;
}


/**
 * @brief Set delay in milliseconds
 *        SysTick_Handler used
 *
 * @param delay - delay in milliseconds
 */
void delayMs(uint64_t delay)
{
    volatile uint64_t tickStart = getSysTick();

    while((getSysTick() - tickStart) < delay);
}

/**
 * @brief Systick ISR
 */
extern "C" void SysTick_Handler(void)
{
    sysTick++;
}
