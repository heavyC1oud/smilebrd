/**
 * @file stm32l475_pin_reg.h
 *
 *       General-purpose I/Os
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_PIN_CPP_H
#define STM32L475_PIN_CPP_H

#include "regs.h"
#include "stm32l475_pin_reg.h"


typedef enum
{
    INPUT,                   //!< input floating
    INPUT_PULLUP,            //!< input pull-up
    INPUT_PULLDOWN,          //!< input pull-down
    ANALOG,                  //!< analog
    OUTPUT,                  //!< output push-pull
    OUTPUT_OD,               //!< output open-drain
    OUTPUT_OD_PULLUP,        //!< output open-drain pull-up
    OUTPUT_OD_PULLDOWN,      //!< output open-drain pull-down
    ALT_INPUT,               //!< alternate function input
    ALT_INPUT_PULLUP,        //!< alternate function input with pull-up
    ALT_INPUT_PULLDOWN,      //!< alternate function input with pull-down
    ALT_OUTPUT,              //!< alternate function output push-pull
    ALT_OUTPUT_PULLUP,       //!< alternate function output push-pull pull-up
    ALT_OUTPUT_PULLDOWN,     //!< alternate function output push-pull pull-down
    ALT_OUTPUT_OD,           //!< alternate function output open-drain
    ALT_OUTPUT_OD_PULLUP,    //!< alternate function output open-drain pull-up
    ALT_OUTPUT_OD_PULLDOWN   //!< alternate function output open-drain pull-down
} pin_config_t;

typedef enum
{
    MODER_INPUT     = 0,
    MODER_OUTPUT    = 1,
    MODER_ALTERNATE = 2,
    MODER_ANALOG    = 3,
    MODER_MASK      = 3
} pin_mode_t;

typedef enum
{
    PUPDR_NONE     = 0,
    PUPDR_PULLUP   = 1,
    PUPDR_PULLDOWN = 2,
    PUPDR_MASK     = 3
} pin_pullup_mode_t;

typedef enum
{
    PIN_SPEED_VERY_LOW  = 0,
    PIN_SPEED_LOW       = 1,
    PIN_SPEED_MEDIUM    = 2,
    PIN_SPEED_HIGH      = 3,
    PIN_SPEED_MASK      = 3
} pin_speed_t;

typedef enum
{
    PUSH_PULL  = 0,
    OPEN_DRAIN = 1
} pin_output_type_t;

typedef enum
{
    ALT_FUNC0          = 0x00, //!< AF0 Alternate Function mapping
    ALT_FUNC1          = 0x01, //!< AF1 Alternate Function mapping
    ALT_FUNC2          = 0x02, //!< AF2 Alternate Function mapping
    ALT_FUNC3          = 0x03, //!< AF3 Alternate Function mapping
    ALT_FUNC4          = 0x04, //!< AF4 Alternate Function mapping
    ALT_FUNC5          = 0x05, //!< AF5 Alternate Function mapping
    ALT_FUNC6          = 0x06, //!< AF6 Alternate Function mapping
    ALT_FUNC7          = 0x07, //!< AF7 Alternate Function mapping
    ALT_FUNC8          = 0x08, //!< AF8 Alternate Function mapping
    ALT_FUNC9          = 0x09, //!< AF9 Alternate Function mapping
    ALT_FUNC10         = 0x0A, //!< AF10 Alternate Function mapping
    ALT_FUNC11         = 0x0B, //!< AF11 Alternate Function mapping
    ALT_FUNC12         = 0x0C, //!< AF12 Alternate Function mapping
    ALT_FUNC13         = 0x0D, //!< AF13 Alternate Function mapping
    ALT_FUNC14         = 0x0E, //!< AF14 Alternate Function mapping
    ALT_FUNC15         = 0x0F, //!< AF15 Alternate Function mapping
} pin_alternate_func_t;

template<char port, uint32_t pin_no, char activestate = 'H', pin_speed_t speed = PIN_SPEED_HIGH>
struct Pin
{
    static const uint32_t pin = pin_no;
    static const uint32_t shift = pin;
    static const uint32_t shift_x2 = pin * 2;
    static const uint32_t shift_x4 = (pin % 8) * 4;
    static const uint32_t mask = 1UL << shift;
    static const uint32_t mask_x2 = 3UL << shift_x2;
    static const uint32_t mask_x4 = 0xFUL << shift_x4;
    static const uint32_t clearmask = 1UL << (pin + 16);

    static struct pin_base_reg
    {
        gpio_regs_t* operator-> ()
        {
            return
                port == 'A' ?  GPIOA_REGS_ADDR:
                port == 'B' ?  GPIOB_REGS_ADDR:
                port == 'C' ?  GPIOC_REGS_ADDR:
                port == 'D' ?  GPIOD_REGS_ADDR:
                port == 'E' ?  GPIOE_REGS_ADDR:
                port == 'F' ?  GPIOF_REGS_ADDR:
                port == 'G' ?  GPIOG_REGS_ADDR:
                               GPIOH_REGS_ADDR;
        }
    } GPIOx;

    inline static void On(bool set_on = true)
    {
        (activestate == 'H') == set_on ? GPIOx->BSRR.d32 = mask : GPIOx->BSRR.d32 = clearmask;
    }

    inline static void Off()
    {
        On(false);
    }

    inline static void Cpl()
    {
        GPIOx->ODR.d32 ^= mask;
    }

    inline static void Mode(pin_config_t cfg)
    {
        switch(cfg)
        {
        case INPUT:
            SetMode(MODER_INPUT);
            PullNone();
            break;

        case INPUT_PULLUP:
            SetMode(MODER_INPUT);
            PullUp();
            break;

        case INPUT_PULLDOWN:
            SetMode(MODER_INPUT);
            PullDown();
            break;

        case ANALOG:
            SetMode(MODER_ANALOG);
            PullNone();
            break;

        case OUTPUT:
            SetMode(MODER_OUTPUT);
            SetSpeed(speed);
            SetPushPull();
            PullNone();
            break;

        case OUTPUT_OD:
            SetMode(MODER_OUTPUT);
            SetSpeed(speed);
            SetOpenDrain();
            PullNone();
            break;

        case OUTPUT_OD_PULLUP:
            SetMode(MODER_OUTPUT);
            SetSpeed(speed);
            SetOpenDrain();
            PullUp();
            break;

        case OUTPUT_OD_PULLDOWN:
            SetMode(MODER_OUTPUT);
            SetSpeed(speed);
            SetOpenDrain();
            PullDown();
            break;

        case ALT_INPUT:
            SetMode(MODER_ALTERNATE);
            PullNone();
            break;

        case ALT_INPUT_PULLUP:
            SetMode(MODER_ALTERNATE);
            PullUp();
            break;

        case ALT_INPUT_PULLDOWN:
            SetMode(MODER_ALTERNATE);
            PullDown();
            break;

        case ALT_OUTPUT:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetPushPull();
            PullNone();
            break;

        case ALT_OUTPUT_PULLUP:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetPushPull();
            PullUp();
            break;

        case ALT_OUTPUT_PULLDOWN:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetPushPull();
            PullDown();
            break;

        case ALT_OUTPUT_OD:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetOpenDrain();
            PullNone();
            break;

        case ALT_OUTPUT_OD_PULLUP:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetOpenDrain();
            PullUp();
            break;

        case ALT_OUTPUT_OD_PULLDOWN:
            SetMode(MODER_ALTERNATE);
            SetSpeed(speed);
            SetOpenDrain();
            PullDown();
            break;
        }
    }

    inline static void Direct(pin_config_t cfg)
    {
        Mode(cfg);
    }

    inline static void SetPullUp(pin_pullup_mode_t mode)
    {
        GPIOx->PUPDR.d32 = (GPIOx->PUPDR.d32 & ~(PUPDR_MASK << shift_x2)) | (mode << shift_x2);
    }

    inline static void PullUp()
    {
        SetPullUp(PUPDR_PULLUP);
    }

    inline static void PullDown()
    {
        SetPullUp(PUPDR_PULLDOWN);
    }

    inline static void PullNone()
    {
        SetPullUp(PUPDR_NONE);
    }

    inline static void SetSpeed(pin_speed_t sp)
    {
        GPIOx->OSPEEDR.d32 = (GPIOx->OSPEEDR.d32 & ~mask_x2) | (sp << shift_x2);
    }

    inline static void SetMode(pin_mode_t mode)
    {
        GPIOx->MODER.d32 = (GPIOx->MODER.d32 & ~mask_x2) | (mode << shift_x2);
    }

    inline static void SetOutputType(pin_output_type_t val)
    {
        GPIOx->OTYPER.d32 = (GPIOx->OTYPER.d32 & ~mask) | (val << shift);
    }

    inline static void SetPushPull()
    {
        SetOutputType(PUSH_PULL);
    }

    inline static void SetOpenDrain()
    {
        SetOutputType(OPEN_DRAIN);
    }

    inline static void Alternate(pin_alternate_func_t val)
    {
        if(pin_no < 8) {
            GPIOx->AFRL.d32 = (GPIOx->AFRL.d32 & ~mask_x4) | (val << shift_x4);
        }
        else {
            GPIOx->AFRH.d32 = (GPIOx->AFRH.d32 & ~mask_x4) | (val << shift_x4);
        }
    }

    inline static uint32_t Latched()
    {
        uint32_t ret = GPIOx->ODR.d32 & mask;
        return activestate == 'L' ? !ret : ret;
    }

    inline static uint32_t Signalled()
    {
        uint32_t ret = GPIOx->IDR.d32 & mask;
        return activestate == 'L' ? !ret : ret;
    }
};

#endif  // STM32L475_PIN_CPP_H
