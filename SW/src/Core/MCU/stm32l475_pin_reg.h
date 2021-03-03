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

#ifndef STM32L475_PIN_REG_H
#define STM32L475_PIN_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief GPIO port mode register (GPIOx_MODER)
 *
 *        Address offset: 0x00
 *        Reset value: 0xABFF FFFF (for port A)
 *        Reset value: 0xFFFF FEBF (for port B)
 *        Reset value: 0xFFFF FFFF (for ports C..G)
 *        Reset value: 0x0000 000F (for port H)
 */
MCU_REG_TYPE(
    gpio_moder_t,
    {
        mcu_reg_t moder0         :  2; /*!< 1:0 MODE0[1:0] Port configuration I/O pin 0 */
        mcu_reg_t moder1         :  2; /*!< 3:2 MODE1[1:0] Port configuration I/O pin 1 */
        mcu_reg_t moder2         :  2; /*!< 5:4 MODE2[1:0] Port configuration I/O pin 2 */
        mcu_reg_t moder3         :  2; /*!< 7:6 MODE3[1:0] Port configuration I/O pin 3 */
        mcu_reg_t moder4         :  2; /*!< 9:8 MODE4[1:0] Port configuration I/O pin 4 */
        mcu_reg_t moder5         :  2; /*!< 11:10 MODE5[1:0] Port configuration I/O pin 5 */
        mcu_reg_t moder6         :  2; /*!< 13:12 MODE6[1:0] Port configuration I/O pin 6 */
        mcu_reg_t moder7         :  2; /*!< 15:14 MODE7[1:0] Port configuration I/O pin 7 */
        mcu_reg_t moder8         :  2; /*!< 17:16 MODE8[1:0] Port configuration I/O pin 8 */
        mcu_reg_t moder9         :  2; /*!< 19:18 MODE9[1:0] Port configuration I/O pin 9 */
        mcu_reg_t moder10        :  2; /*!< 21:20 MODE10[1:0] Port configuration I/O pin 10 */
        mcu_reg_t moder11        :  2; /*!< 23:22 MODE11[1:0] Port configuration I/O pin 11 */
        mcu_reg_t moder12        :  2; /*!< 25:24 MODE12[1:0] Port configuration I/O pin 12 */
        mcu_reg_t moder13        :  2; /*!< 27:26 MODE13[1:0] Port configuration I/O pin 13 */
        mcu_reg_t moder14        :  2; /*!< 29:28 MODE14[1:0] Port configuration I/O pin 14 */
        mcu_reg_t moder15        :  2; /*!< 31:30 MODE15[1:0] Port configuration I/O pin 15 */
    }
);

/**
 * @brief GPIO port output type register (GPIOx_OTYPER)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_otyper_t,
    {
        mcu_reg_t ot0            :  1; /*!< 0 OT0 Port configuration I/O pin 0 */
        mcu_reg_t ot1            :  1; /*!< 1 OT1 Port configuration I/O pin 1 */
        mcu_reg_t ot2            :  1; /*!< 2 OT2 Port configuration I/O pin 2 */
        mcu_reg_t ot3            :  1; /*!< 3 OT3 Port configuration I/O pin 3 */
        mcu_reg_t ot4            :  1; /*!< 4 OT4 Port configuration I/O pin 4 */
        mcu_reg_t ot5            :  1; /*!< 5 OT5 Port configuration I/O pin 5 */
        mcu_reg_t ot6            :  1; /*!< 6 OT6 Port configuration I/O pin 6 */
        mcu_reg_t ot7            :  1; /*!< 7 OT7 Port configuration I/O pin 7 */
        mcu_reg_t ot8            :  1; /*!< 8 OT8 Port configuration I/O pin 8 */
        mcu_reg_t ot9            :  1; /*!< 9 OT9 Port configuration I/O pin 9 */
        mcu_reg_t ot10           :  1; /*!< 10 OT10 Port configuration I/O pin 10 */
        mcu_reg_t ot11           :  1; /*!< 11 OT11 Port configuration I/O pin 11 */
        mcu_reg_t ot12           :  1; /*!< 12 OT12 Port configuration I/O pin 12 */
        mcu_reg_t ot13           :  1; /*!< 13 OT13 Port configuration I/O pin 13 */
        mcu_reg_t ot14           :  1; /*!< 14 OT14 Port configuration I/O pin 14 */
        mcu_reg_t ot15           :  1; /*!< 15 OT15 Port configuration I/O pin 15 */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO port output speed register (GPIOx_OSPEEDR)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0C00 0000 (for port A)
 *        Reset value: 0x0000 0000 (for the other ports)
 */
MCU_REG_TYPE(
    gpio_ospeedr_t,
    {
        mcu_reg_t ospeedr0       :  2; /*!< 1:0 OSPEED0[1:0] Port configuration I/O pin 0 */
        mcu_reg_t ospeedr1       :  2; /*!< 3:2 OSPEED1[1:0] Port configuration I/O pin 1 */
        mcu_reg_t ospeedr2       :  2; /*!< 5:4 OSPEED2[1:0] Port configuration I/O pin 2 */
        mcu_reg_t ospeedr3       :  2; /*!< 7:6 OSPEED3[1:0] Port configuration I/O pin 3 */
        mcu_reg_t ospeedr4       :  2; /*!< 9:8 OSPEED4[1:0] Port configuration I/O pin 4 */
        mcu_reg_t ospeedr5       :  2; /*!< 11:10 OSPEED5[1:0] Port configuration I/O pin 5 */
        mcu_reg_t ospeedr6       :  2; /*!< 13:12 OSPEED6[1:0] Port configuration I/O pin 6 */
        mcu_reg_t ospeedr7       :  2; /*!< 15:14 OSPEED7[1:0] Port configuration I/O pin 7 */
        mcu_reg_t ospeedr8       :  2; /*!< 17:16 OSPEED8[1:0] Port configuration I/O pin 8 */
        mcu_reg_t ospeedr9       :  2; /*!< 19:18 OSPEED9[1:0] Port configuration I/O pin 9 */
        mcu_reg_t ospeedr10      :  2; /*!< 21:20 OSPEED10[1:0] Port configuration I/O pin 10 */
        mcu_reg_t ospeedr11      :  2; /*!< 23:22 OSPEED11[1:0] Port configuration I/O pin 11 */
        mcu_reg_t ospeedr12      :  2; /*!< 25:24 OSPEED12[1:0] Port configuration I/O pin 12 */
        mcu_reg_t ospeedr13      :  2; /*!< 27:26 OSPEED13[1:0] Port configuration I/O pin 13 */
        mcu_reg_t ospeedr14      :  2; /*!< 29:28 OSPEED14[1:0] Port configuration I/O pin 14 */
        mcu_reg_t ospeedr15      :  2; /*!< 31:30 OSPEED15[1:0] Port configuration I/O pin 15 */
    }
);

/**
 * @brief GPIO port pull-up/pull-down register (GPIOx_PUPDR)
 *
 *        Address offset: 0x0C
 *        Reset value: 0x6400 0000 (for port A)
 *        Reset value: 0x0000 0100 (for port B)
 *        Reset value: 0x0000 0000 (for other ports)
 */
MCU_REG_TYPE(
    gpio_pupdr_t,
    {
        mcu_reg_t pupdr0         :  2; /*!< 1:0 PUPD0[1:0] Port configuration I/O pin 0 */
        mcu_reg_t pupdr1         :  2; /*!< 3:2 PUPD1[1:0] Port configuration I/O pin 1 */
        mcu_reg_t pupdr2         :  2; /*!< 5:4 PUPD2[1:0] Port configuration I/O pin 2 */
        mcu_reg_t pupdr3         :  2; /*!< 7:6 PUPD3[1:0] Port configuration I/O pin 3 */
        mcu_reg_t pupdr4         :  2; /*!< 9:8 PUPD4[1:0] Port configuration I/O pin 4 */
        mcu_reg_t pupdr5         :  2; /*!< 11:10 PUPD5[1:0] Port configuration I/O pin 5 */
        mcu_reg_t pupdr6         :  2; /*!< 13:12 PUPD6[1:0] Port configuration I/O pin 6 */
        mcu_reg_t pupdr7         :  2; /*!< 15:14 PUPD7[1:0] Port configuration I/O pin 7 */
        mcu_reg_t pupdr8         :  2; /*!< 17:16 PUPD8[1:0] Port configuration I/O pin 8 */
        mcu_reg_t pupdr9         :  2; /*!< 19:18 PUPD9[1:0] Port configuration I/O pin 9 */
        mcu_reg_t pupdr10        :  2; /*!< 21:20 PUPD10[1:0] Port configuration I/O pin 10 */
        mcu_reg_t pupdr11        :  2; /*!< 23:22 PUPD11[1:0] Port configuration I/O pin 11 */
        mcu_reg_t pupdr12        :  2; /*!< 25:24 PUPD12[1:0] Port configuration I/O pin 12 */
        mcu_reg_t pupdr13        :  2; /*!< 27:26 PUPD13[1:0] Port configuration I/O pin 13 */
        mcu_reg_t pupdr14        :  2; /*!< 29:28 PUPD14[1:0] Port configuration I/O pin 14 */
        mcu_reg_t pupdr15        :  2; /*!< 31:30 PUPD15[1:0] Port configuration I/O pin 15 */
    }
);

/**
 * @brief GPIO port input data register (GPIOx_IDR)
 *
 *        Address offset: 0x10
 *        Reset value: 0x0000 XXXX
 */
MCU_REG_TYPE(
    gpio_idr_t,
    {
        mcu_reg_t idr0           :  1; /*!< 0 ID0 Port input data I/O pin 0 */
        mcu_reg_t idr1           :  1; /*!< 1 ID1 Port input data I/O pin 1 */
        mcu_reg_t idr2           :  1; /*!< 2 ID2 Port input data I/O pin 2 */
        mcu_reg_t idr3           :  1; /*!< 3 ID3 Port input data I/O pin 3 */
        mcu_reg_t idr4           :  1; /*!< 4 ID4 Port input data I/O pin 4 */
        mcu_reg_t idr5           :  1; /*!< 5 ID5 Port input data I/O pin 5 */
        mcu_reg_t idr6           :  1; /*!< 6 ID6 Port input data I/O pin 6 */
        mcu_reg_t idr7           :  1; /*!< 7 ID7 Port input data I/O pin 7 */
        mcu_reg_t idr8           :  1; /*!< 8 ID8 Port input data I/O pin 8 */
        mcu_reg_t idr9           :  1; /*!< 9 ID9 Port input data I/O pin 9 */
        mcu_reg_t idr10          :  1; /*!< 10 ID10 Port input data I/O pin 10 */
        mcu_reg_t idr11          :  1; /*!< 11 ID11 Port input data I/O pin 11 */
        mcu_reg_t idr12          :  1; /*!< 12 ID12 Port input data I/O pin 12 */
        mcu_reg_t idr13          :  1; /*!< 13 ID13 Port input data I/O pin 13 */
        mcu_reg_t idr14          :  1; /*!< 14 ID14 Port input data I/O pin 14 */
        mcu_reg_t idr15          :  1; /*!< 15 ID15 Port input data I/O pin 15 */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO port output data register (GPIOx_ODR)
 *
 *        Address offset: 0x14
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_odr_t,
    {
        mcu_reg_t odr0           :  1; /*!< 0 OD0 Port output data I/O pin 0 */
        mcu_reg_t odr1           :  1; /*!< 1 OD1 Port output data I/O pin 1 */
        mcu_reg_t odr2           :  1; /*!< 2 OD2 Port output data I/O pin 2 */
        mcu_reg_t odr3           :  1; /*!< 3 OD3 Port output data I/O pin 3 */
        mcu_reg_t odr4           :  1; /*!< 4 OD4 Port output data I/O pin 4 */
        mcu_reg_t odr5           :  1; /*!< 5 OD5 Port output data I/O pin 5 */
        mcu_reg_t odr6           :  1; /*!< 6 OD6 Port output data I/O pin 6 */
        mcu_reg_t odr7           :  1; /*!< 7 OD7 Port output data I/O pin 7 */
        mcu_reg_t odr8           :  1; /*!< 8 OD8 Port output data I/O pin 8 */
        mcu_reg_t odr9           :  1; /*!< 9 OD9 Port output data I/O pin 9 */
        mcu_reg_t odr10          :  1; /*!< 10 OD10 Port output data I/O pin 10 */
        mcu_reg_t odr11          :  1; /*!< 11 OD11 Port output data I/O pin 11 */
        mcu_reg_t odr12          :  1; /*!< 12 OD12 Port output data I/O pin 12 */
        mcu_reg_t odr13          :  1; /*!< 13 OD13 Port output data I/O pin 13 */
        mcu_reg_t odr14          :  1; /*!< 14 OD14 Port output data I/O pin 14 */
        mcu_reg_t odr15          :  1; /*!< 15 OD15 Port output data I/O pin 15 */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO port bit set/reset register (GPIOx_BSRR)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_bsrr_t,
    {
        mcu_reg_t bs0            :  1; /*!< 0 BS0 Port set bit y I/O pin 0 */
        mcu_reg_t bs1            :  1; /*!< 1 BS1 Port set bit y I/O pin 1 */
        mcu_reg_t bs2            :  1; /*!< 2 BS2 Port set bit y I/O pin 2 */
        mcu_reg_t bs3            :  1; /*!< 3 BS3 Port set bit y I/O pin 3 */
        mcu_reg_t bs4            :  1; /*!< 4 BS4 Port set bit y I/O pin 4 */
        mcu_reg_t bs5            :  1; /*!< 5 BS5 Port set bit y I/O pin 5 */
        mcu_reg_t bs6            :  1; /*!< 6 BS6 Port set bit y I/O pin 6 */
        mcu_reg_t bs7            :  1; /*!< 7 BS7 Port set bit y I/O pin 7 */
        mcu_reg_t bs8            :  1; /*!< 8 BS8 Port set bit y I/O pin 8 */
        mcu_reg_t bs9            :  1; /*!< 9 BS9 Port set bit y I/O pin 9 */
        mcu_reg_t bs10           :  1; /*!< 10 BS10 Port set bit y I/O pin 10 */
        mcu_reg_t bs11           :  1; /*!< 11 BS11 Port set bit y I/O pin 11 */
        mcu_reg_t bs12           :  1; /*!< 12 BS12 Port set bit y I/O pin 12 */
        mcu_reg_t bs13           :  1; /*!< 13 BS13 Port set bit y I/O pin 13 */
        mcu_reg_t bs14           :  1; /*!< 14 BS14 Port set bit y I/O pin 14 */
        mcu_reg_t bs15           :  1; /*!< 15 BS15 Port set bit y I/O pin 15 */
        mcu_reg_t br0            :  1; /*!< 16 BR0 Port reset bit y I/O pin 0 */
        mcu_reg_t br1            :  1; /*!< 17 BR1 Port reset bit y I/O pin 1 */
        mcu_reg_t br2            :  1; /*!< 18 BR2 Port reset bit y I/O pin 2 */
        mcu_reg_t br3            :  1; /*!< 19 BR3 Port reset bit y I/O pin 3 */
        mcu_reg_t br4            :  1; /*!< 20 BR4 Port reset bit y I/O pin 4 */
        mcu_reg_t br5            :  1; /*!< 21 BR5 Port reset bit y I/O pin 5 */
        mcu_reg_t br6            :  1; /*!< 22 BR6 Port reset bit y I/O pin 6 */
        mcu_reg_t br7            :  1; /*!< 23 BR7 Port reset bit y I/O pin 7 */
        mcu_reg_t br8            :  1; /*!< 24 BR8 Port reset bit y I/O pin 8 */
        mcu_reg_t br9            :  1; /*!< 25 BR9 Port reset bit y I/O pin 9 */
        mcu_reg_t br10           :  1; /*!< 26 BR10 Port reset bit y I/O pin 10 */
        mcu_reg_t br11           :  1; /*!< 27 BR11 Port reset bit y I/O pin 11 */
        mcu_reg_t br12           :  1; /*!< 28 BR12 Port reset bit y I/O pin 12 */
        mcu_reg_t br13           :  1; /*!< 29 BR13 Port reset bit y I/O pin 13 */
        mcu_reg_t br14           :  1; /*!< 30 BR14 Port reset bit y I/O pin 14 */
        mcu_reg_t br15           :  1; /*!< 31 BR15 Port reset bit y I/O pin 15 */
    }
);

/**
 * @brief GPIO port configuration lock register (GPIOxLCKR)
 *
 *        Address offset: 0x1C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_lckr_t,
    {
        mcu_reg_t lck0           :  1; /*!< 0 LCK0 Port lock bit y I/O pin 0 */
        mcu_reg_t lck1           :  1; /*!< 1 LCK1 Port lock bit y I/O pin 1 */
        mcu_reg_t lck2           :  1; /*!< 2 LCK2 Port lock bit y I/O pin 2 */
        mcu_reg_t lck3           :  1; /*!< 3 LCK3 Port lock bit y I/O pin 3 */
        mcu_reg_t lck4           :  1; /*!< 4 LCK4 Port lock bit y I/O pin 4 */
        mcu_reg_t lck5           :  1; /*!< 5 LCK5 Port lock bit y I/O pin 5 */
        mcu_reg_t lck6           :  1; /*!< 6 LCK6 Port lock bit y I/O pin 6 */
        mcu_reg_t lck7           :  1; /*!< 7 LCK7 Port lock bit y I/O pin 7 */
        mcu_reg_t lck8           :  1; /*!< 8 LCK8 Port lock bit y I/O pin 8 */
        mcu_reg_t lck9           :  1; /*!< 9 LCK9 Port lock bit y I/O pin 9 */
        mcu_reg_t lck10          :  1; /*!< 10 LCK10 Port lock bit y I/O pin 10 */
        mcu_reg_t lck11          :  1; /*!< 11 LCK11 Port lock bit y I/O pin 11 */
        mcu_reg_t lck12          :  1; /*!< 12 LCK12 Port lock bit y I/O pin 12 */
        mcu_reg_t lck13          :  1; /*!< 13 LCK13 Port lock bit y I/O pin 13 */
        mcu_reg_t lck14          :  1; /*!< 14 LCK14 Port lock bit y I/O pin 14 */
        mcu_reg_t lck15          :  1; /*!< 15 LCK15 Port lock bit y I/O pin 15 */
        mcu_reg_t lckk           :  1; /*!< 16 Lock key */
        mcu_reg_t Reserved17_31  : 15; /*!< 31:17 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO alternate function low register (GPIOx_AFRL)
 *
 *        Address offset: 0x20
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_afrl_t,
    {
        mcu_reg_t afsel0         :  4; /*!< 3:0 AFSEL0[3:0] Alternate function selection for port bit 0 */
        mcu_reg_t afsel1         :  4; /*!< 7:4 AFSEL1[3:0] Alternate function selection for port bit 1 */
        mcu_reg_t afsel2         :  4; /*!< 11:8 AFSEL2[3:0] Alternate function selection for port bit 2 */
        mcu_reg_t afsel3         :  4; /*!< 15:12 AFSEL3[3:0] Alternate function selection for port bit 3 */
        mcu_reg_t afsel4         :  4; /*!< 19:16 AFSEL4[3:0] Alternate function selection for port bit 4 */
        mcu_reg_t afsel5         :  4; /*!< 23:20 AFSEL5[3:0] Alternate function selection for port bit 5 */
        mcu_reg_t afsel6         :  4; /*!< 27:24 AFSEL6[3:0] Alternate function selection for port bit 6 */
        mcu_reg_t afsel7         :  4; /*!< 31:28 AFSEL7[3:0] Alternate function selection for port bit 7 */
    }
);

/**
 * @brief GPIO alternate function high register (GPIOx_AFRH)
 *
 *        Address offset: 0x24
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_afrh_t,
    {
        mcu_reg_t afsel8         :  4; /*!< 3:0 AFSEL8[3:0] Alternate function selection for port x bit 0 */
        mcu_reg_t afsel9         :  4; /*!< 7:4 AFSEL9[3:0] Alternate function selection for port x bit 1 */
        mcu_reg_t afsel10        :  4; /*!< 11:8 AFSEL10[3:0] Alternate function selection for port x bit 2 */
        mcu_reg_t afsel11        :  4; /*!< 15:12 AFSEL11[3:0] Alternate function selection for port x bit 3 */
        mcu_reg_t afsel12        :  4; /*!< 19:16 AFSEL12[3:0] Alternate function selection for port x bit 4 */
        mcu_reg_t afsel13        :  4; /*!< 23:20 AFSEL13[3:0] Alternate function selection for port x bit 5 */
        mcu_reg_t afsel14        :  4; /*!< 27:24 AFSEL14[3:0] Alternate function selection for port x bit 6 */
        mcu_reg_t afsel15        :  4; /*!< 31:28 AFSEL15[3:0] Alternate function selection for port x bit 7 */
    }
);

/**
 * @brief GPIO port bit reset register (GPIOx_BRR)
 *
 *        Address offset: 0x28
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_brr_t,
    {
        mcu_reg_t br0            :  1; /*!< 0 BR0 Port Reset bit */
        mcu_reg_t br1            :  1; /*!< 1 BR1 Port Reset bit */
        mcu_reg_t br2            :  1; /*!< 2 BR2 Port Reset bit */
        mcu_reg_t br3            :  1; /*!< 3 BR3 Port Reset bit */
        mcu_reg_t br4            :  1; /*!< 4 BR4 Port Reset bit */
        mcu_reg_t br5            :  1; /*!< 5 BR5 Port Reset bit */
        mcu_reg_t br6            :  1; /*!< 6 BR6 Port Reset bit */
        mcu_reg_t br7            :  1; /*!< 7 BR7 Port Reset bit */
        mcu_reg_t br8            :  1; /*!< 8 BR8 Port Reset bit */
        mcu_reg_t br9            :  1; /*!< 9 BR9 Port Reset bit */
        mcu_reg_t br10           :  1; /*!< 10 BR10 Port Reset bit */
        mcu_reg_t br11           :  1; /*!< 11 BR11 Port Reset bit */
        mcu_reg_t br12           :  1; /*!< 12 BR12 Port Reset bit */
        mcu_reg_t br13           :  1; /*!< 13 BR13 Port Reset bit */
        mcu_reg_t br14           :  1; /*!< 14 BR14 Port Reset bit */
        mcu_reg_t br15           :  1; /*!< 15 BR15 Port Reset bit */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO port analog switch control register (GPIOx_ASCR)
 *
 *        Address offset: 0x2C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE(
    gpio_ascr_t,
    {
        mcu_reg_t asc0            :  1; /*!< 0 ASC0 Port analog switch control I/O pin 0 */
        mcu_reg_t asc1            :  1; /*!< 1 ASC1 Port analog switch control I/O pin 1 */
        mcu_reg_t asc2            :  1; /*!< 2 ASC2 Port analog switch control I/O pin 2 */
        mcu_reg_t asc3            :  1; /*!< 3 ASC3 Port analog switch control I/O pin 3 */
        mcu_reg_t asc4            :  1; /*!< 4 ASC4 Port analog switch control I/O pin 4 */
        mcu_reg_t asc5            :  1; /*!< 5 ASC5 Port analog switch control I/O pin 5 */
        mcu_reg_t asc6            :  1; /*!< 6 ASC6 Port analog switch control I/O pin 6 */
        mcu_reg_t asc7            :  1; /*!< 7 ASC7 Port analog switch control I/O pin 7 */
        mcu_reg_t asc8            :  1; /*!< 8 ASC8 Port analog switch control I/O pin 8 */
        mcu_reg_t asc9            :  1; /*!< 9 ASC9 Port analog switch control I/O pin 9 */
        mcu_reg_t asc10           :  1; /*!< 10 ASC10 Port analog switch control I/O pin 10 */
        mcu_reg_t asc11           :  1; /*!< 11 ASC11 Port analog switch control I/O pin 11 */
        mcu_reg_t asc12           :  1; /*!< 12 ASC12 Port analog switch control I/O pin 12 */
        mcu_reg_t asc13           :  1; /*!< 13 ASC13 Port analog switch control I/O pin 13 */
        mcu_reg_t asc14           :  1; /*!< 14 ASC14 Port analog switch control I/O pin 14 */
        mcu_reg_t asc15           :  1; /*!< 15 ASC15 Port analog switch control I/O pin 15 */
        mcu_reg_t Reserved16_31   : 16; /*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief GPIO register map
 */
typedef __packed struct
{
    volatile gpio_moder_t      MODER;       /*!< 001 0X00 */
    volatile gpio_otyper_t     OTYPER;      /*!< 002 0X04 */
    volatile gpio_ospeedr_t    OSPEEDR;     /*!< 003 0X08 */
    volatile gpio_pupdr_t      PUPDR;       /*!< 004 0X0C */
    volatile gpio_idr_t        IDR;         /*!< 005 0X10 */
    volatile gpio_odr_t        ODR;         /*!< 006 0X14 */
    volatile gpio_bsrr_t       BSRR;        /*!< 007 0X18 */
    volatile gpio_lckr_t       LCKR;        /*!< 008 0X1C */
    volatile gpio_afrl_t       AFRL;        /*!< 009 0X20 */
    volatile gpio_afrh_t       AFRH;        /*!< 010 0X24 */
    volatile gpio_brr_t        BRR;         /*!< 011 0X28 */
    volatile gpio_ascr_t       ASCR;        /*!< 012 0X2C */
} gpio_regs_t;

CHECK_REG_MAP(gpio_regs_t, 12);

#define GPIOA_REGS_ADDR ((gpio_regs_t*)GPIOA_BASE)
#define GPIOB_REGS_ADDR ((gpio_regs_t*)GPIOB_BASE)
#define GPIOC_REGS_ADDR ((gpio_regs_t*)GPIOC_BASE)
#define GPIOD_REGS_ADDR ((gpio_regs_t*)GPIOD_BASE)
#define GPIOE_REGS_ADDR ((gpio_regs_t*)GPIOE_BASE)
#define GPIOF_REGS_ADDR ((gpio_regs_t*)GPIOF_BASE)
#define GPIOG_REGS_ADDR ((gpio_regs_t*)GPIOG_BASE)
#define GPIOH_REGS_ADDR ((gpio_regs_t*)GPIOH_BASE)

#endif  // STM32L475_PIN_REG_H
