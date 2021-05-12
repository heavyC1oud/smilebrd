/**
 * @file stm32l475_tsc_reg.h
 *
 *       Touch sensing controller registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_TSC_REG_H
#define STM32L475_TSC_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief TSC control register (TSC_CR)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_cr_t,
    {
        mcu_reg_t tsce           : 1; /*!< 0 TSCE: Touch sensing controller enable */
        mcu_reg_t start          : 1; /*!< 1 START: Start a new acquisition */
        mcu_reg_t am             : 1; /*!< 2 AM: Acquisition mode */
        mcu_reg_t syncpol        : 1; /*!< 3 SYNCPOL: Synchronization pin polarity */
        mcu_reg_t iodef          : 1; /*!< 4 IODEF: I/O Default mode */
        mcu_reg_t mcv            : 3; /*!< 7:5 MCV[2:0] Max count value */
        mcu_reg_t Reserved8_11   : 4; /*!< 11:8 Reserved, must be kept at reset value.*/
        mcu_reg_t pgpsc          : 3; /*!< 14:12 PGPSC[2:0] Pulse generator prescaler */
        mcu_reg_t sspsc          : 1; /*!< 15 SSPSC: Spread spectrum prescaler */
        mcu_reg_t sse            : 1; /*!< 16 SSE: Spread spectrum enable */
        mcu_reg_t ssd            : 7; /*!< 23:17 SSD[6:0] Spread spectrum deviation */
        mcu_reg_t ctpl           : 4; /*!< 27:24 CTPL[3:0] Charge transfer pulse low */
        mcu_reg_t ctph           : 4; /*!< 31:28 CTPH[3:0] Charge transfer pulse high */
    }
);

/**
 * @brief TSC interrupt enable register (TSC_IER)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_ier_t,
    {
        mcu_reg_t eoaie          : 1; /*!< 0 EOAIE: End of acquisition interrupt enable */
        mcu_reg_t mceie          : 1; /*!< 1 MCEIE: Max count error interrupt enable */
        mcu_reg_t Reserved2_31   : 30; /*!< 31:2 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief TSC interrupt clear register (TSC_ICR)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_icr_t,
    {
        mcu_reg_t eoaic          : 1; /*!< 0 EOAIC: End of acquisition interrupt clear */
        mcu_reg_t mceic          : 1; /*!< 1 MCEIC: Max count error interrupt clear */
        mcu_reg_t Reserved2_31   : 30; /*!< 31:2 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief TSC interrupt status register (TSC_ISR)
 *
 *        Address offset: 0x0C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_isr_t,
    {
        mcu_reg_t eoaf           : 1; /*!< 0 EOAF: End of acquisition flag */
        mcu_reg_t mcef           : 1; /*!< 1 MCEC: Max count error flag */
        mcu_reg_t Reserved2_31   : 30; /*!< 31:2 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief TSC I/O hysteresis control register (TSC_IOHCR)
 *
 *        Address offset: 0x10
 *        Reset value: 0xFFFF FFFF
 */
MCU_REG_TYPE (
    tsc_iohcr_t,
    {
        mcu_reg_t gxioy          : 32; /*!< 31:0 Gx_IOy: Gx_IOy Schmitt trigger hysteresis mode */
    }
);

/**
 * @brief TSC I/O analog switch control register (TSC_IOASCR)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_ioascr_t,
    {
        mcu_reg_t gxioy          : 32; /*!< 31:0 Gx_IOy: Gx_IOy analog switch enable */
    }
);

/**
 * @brief TSC I/O sampling control register (TSC_IOSCR)
 *
 *        Address offset: 0x20
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_ioscr_t,
    {
        mcu_reg_t gxioy          : 32; /*!< 31:0 Gx_IOy: Gx_IOy sampling mode */
    }
);

/**
 * @brief TSC I/O sampling control register (TSC_IOCCR)
 *
 *        Address offset: 0x28
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_ioccr_t,
    {
        mcu_reg_t gxioy          : 32; /*!< 31:0 Gx_IOy: Gx_IOy channel mode */
    }
);

/**
 * @brief TSC I/O group control register (TSC_IOGCSR)
 *
 *        Address offset: 0x30
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_iogcsr_t,
    {
        mcu_reg_t gxe            : 8; /*!< 7:0 GxE: Analog I/O group x enable */
        mcu_reg_t Reserved8_15   : 8; /*!< 15:8 Reserved, must be kept at reset value.*/
        mcu_reg_t gxs            : 8; /*!< 23:16 GxE: Analog I/O group x enable */
        mcu_reg_t Reserved24_31  : 8; /*!< 31:24 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief TSC I/O group x counter register (TSC_IOGxCR)
 *
 *        x represents the analog I/O group number
 *        Address offset: 0x30 + 0x04 * x, (x = 1..8)
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tsc_iogxcr_t,
    {
        mcu_reg_t cnt            : 14; /*!< 13:0 CNT[13:0] Counter value */
        mcu_reg_t Reserved14_31  : 18; /*!< 31:14 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief TSC register map
 */
typedef __packed struct
{
    volatile tsc_cr_t      CR;            /*!< 1 0x00 */
    volatile tsc_ier_t     IER;           /*!< 2 0x04 */
    volatile tsc_icr_t     ICR;           /*!< 3 0x08 */
    volatile tsc_isr_t     ISR;           /*!< 4 0x0C */
    volatile tsc_iohcr_t   IOHCR;         /*!< 5 0x10 */
    volatile mcu_reg_t     RESERVED0;     /*!< 6 0x14 */
    volatile tsc_ioascr_t  IOASCR;        /*!< 7 0x18 */
    volatile mcu_reg_t     RESERVED1;     /*!< 8 0x1C */
    volatile tsc_ioscr_t   IOSCR;         /*!< 9 0x20 */
    volatile mcu_reg_t     RESERVED2;     /*!< 10 0x24 */
    volatile tsc_ioccr_t   IOCCR;         /*!< 11 0x28 */
    volatile mcu_reg_t     RESERVED3;     /*!< 12 0x2C */
    volatile tsc_iogcsr_t  IOGCSR;        /*!< 13 0x30 */
    volatile tsc_iogxcr_t  IOGxCR[8];     /*!< 14..21 0x34..0x50 */

} tsc_regs_t;

CHECK_REG_MAP(tsc_regs_t, (13 + 8));

#define TSC_REGS_ADDR (tsc_regs_t*)TSC_BASE

#endif  // STM32L475_TSC_REG_H
