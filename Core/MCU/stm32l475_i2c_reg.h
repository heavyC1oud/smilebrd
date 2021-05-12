/**
 * @file stm32l475_i2c_reg.h
 *
 *       Inter-integrated circuit registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_I2C_REG_H
#define STM32L475_I2C_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief I2C control register 1 (I2C_CR1)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_cr1_t,
    {
        mcu_reg_t pe             : 1; /*!< 0 PE: Peripheral enable */
        mcu_reg_t txie           : 1; /*!< 1 TXIE: TX Interrupt enable */
        mcu_reg_t rxie           : 1; /*!< 2 RXIE: RX Interrupt enable */
        mcu_reg_t addrie         : 1; /*!< 3 ADDRIE: Address match interrupt enable (slave only) */
        mcu_reg_t nackie         : 1; /*!< 4 NACKIE: Not acknowledge interrupt enable */
        mcu_reg_t stopie         : 1; /*!< 5 STOPIE: Stop detection interrupt enable */
        mcu_reg_t tcie           : 1; /*!< 6 TCIE: Transfer Complete interrupt enable */
        mcu_reg_t errie          : 1; /*!< 7 ERRIE: Error interrupts enable */
        mcu_reg_t dnf            : 4; /*!< 11:8 DNF[3:0] Digital noise filter */
        mcu_reg_t anfoff         : 1; /*!< 12 ANFOFF: Analog noise filter OFF */
        mcu_reg_t Reserved13     : 1; /*!< 13 Reserved, must be kept at reset value.*/
        mcu_reg_t txdmaen        : 1; /*!< 14 TXDMAEN: DMA transmission requests enable */
        mcu_reg_t rxdmaen        : 1; /*!< 15 RXDMAEN: DMA reception requests enable */
        mcu_reg_t sbc            : 1; /*!< 16 SBC: Slave byte control */
        mcu_reg_t nostretch      : 1; /*!< 17 NOSTRETCH: Clock stretching disable */
        mcu_reg_t wupen          : 1; /*!< 18 WUPEN: Wakeup from Stop mode enable */
        mcu_reg_t gcen           : 1; /*!< 19 GCEN: General call enable */
        mcu_reg_t smbhen         : 1; /*!< 20 SMBHEN: SMBus Host Address enable */
        mcu_reg_t smbden         : 1; /*!< 21 SMBDEN: SMBus Device Default Address enable */
        mcu_reg_t alerten        : 1; /*!< 22 ALERTEN: SMBus alert enable */
        mcu_reg_t pecen          : 1; /*!< 23 PECEN: PEC enable */
        mcu_reg_t Reserved24_31  : 8; /*!< 31:24 Reserved, must be kept at reset value.*/

    }
);

/**
 * @brief I2C control register 2 (I2C_CR2)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_cr2_t,
    {
        mcu_reg_t sadd           : 10; /*!< 9:0 SADD[9:0] Slave address (master mode) */
        mcu_reg_t rd_wrn         : 1; /*!< 10 RD_WRN: Transfer direction (master mode) */
        mcu_reg_t add10          : 1; /*!< 11 ADD10: 10-bit addressing mode (master mode) */
        mcu_reg_t head10r        : 1; /*!< 12 HEAD10R: 10-bit address header only read direction (master receiver mode) */
        mcu_reg_t start          : 1; /*!< 13 START: Start generation */
        mcu_reg_t stop           : 1; /*!< 14 STOP: Stop generation (master mode)*/
        mcu_reg_t nack           : 1; /*!< 15 NACK: NACK generation (slave mode)*/
        mcu_reg_t nbytes         : 8; /*!< 23:16 NBYTES[7:0] Nymber of bytes */
        mcu_reg_t reload         : 1; /*!< 24 RELOAD: NBYTES reload mode */
        mcu_reg_t autoend        : 1; /*!< 25 AUTOEND: Automatic end mode (master mode) */
        mcu_reg_t pecbyte        : 1; /*!< 26 PECBYTE: Packet error checking byte */
        mcu_reg_t Reserved27_31  : 5; /*!< 31:27 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C own address 1 register (I2C_OAR1)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_oar1_t,
    {
        mcu_reg_t oa1            : 10; /*!< 9:0 OA1[9:0] Interface own slave address */
        mcu_reg_t oa1mode        : 1; /*!< 10 OA1MODE: Own Address 1 10-bit mode */
        mcu_reg_t Reserved11_14  : 4; /*!< 14:11 Reserved, must be kept at reset value.*/
        mcu_reg_t oa1en          : 1; /*!< 15 OA1EN: Own Address 1 enable */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C own address 2 register (I2C_OAR2)
 *
 *        Address offset: 0x0C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_oar2_t,
    {
        mcu_reg_t Reserved0      : 1; /*!< 0 Reserved, must be kept at reset value.*/
        mcu_reg_t oa2            : 7; /*!< 7:1 OA2[7:1] Interface address */
        mcu_reg_t oa2msk         : 3; /*!< 10:8 OA2MSK[2:0] Own Address 2 masks */
        mcu_reg_t Reserved11_14  : 4; /*!< 14:11 Reserved, must be kept at reset value.*/
        mcu_reg_t oa2en          : 1; /*!< 15 OA2EN: Own Address 2 enable */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C timing register (I2C_TIMING)
 *
 *        Address offset: 0x10
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_timing_t,
    {
        mcu_reg_t scll           : 8; /*!< 7:0 SCLL[7:0] SCL low period (master mode) */
        mcu_reg_t sclh           : 8; /*!< 15:8 SCLH[7:0] SCL high period (master mode) */
        mcu_reg_t sdadel         : 4; /*!< 19:16 SDADEL[3:0] Data hold time */
        mcu_reg_t scldel         : 4; /*!< 23:20 SCLDEL[3:0] Data setup time */
        mcu_reg_t Reserved24_27  : 4; /*!< 27:24 Reserved, must be kept at reset value.*/
        mcu_reg_t presc          : 4; /*!< 31:28 PRESC[3:0] Timing prescaler */
    }
);

/**
 * @brief I2C timeout register (I2C_TIMEOUT)
 *
 *        Address offset: 0x14
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_timeout_t,
    {
        mcu_reg_t timeouta       : 12; /*!< 11:0 TIMEOUTA[11:0] Bus Timeout A */
        mcu_reg_t tidle          : 1; /*!< 12 TIDLE: Idle clock timeout detection */
        mcu_reg_t Reserved13_14  : 2; /*!< 14:13 Reserved, must be kept at reset value.*/
        mcu_reg_t timeouten      : 1; /*!< 15 TIMEOUTEN: Clock timeout enable */
        mcu_reg_t timeoutb       : 12; /*!< 27:16 TIMEOUTB[11:0] Bus Timeout B */
        mcu_reg_t Reserved30_28  : 2; /*!< 30:28 Reserved, must be kept at reset value.*/
        mcu_reg_t texten         : 1; /*!< 31 TEXTEN: Extended clock timeout enable */
    }
);

/**
 * @brief I2C interrupt and status register (I2C_ISR)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0001
 */
MCU_REG_TYPE (
    i2c_isr_t,
    {
        mcu_reg_t txe            : 1; /*!< 0 TXE: Transmit data register empty (transmitters) */
        mcu_reg_t txis           : 1; /*!< 1 TXIS: Transmit interrupt status (transmitters) */
        mcu_reg_t rxne           : 1; /*!< 2 RXNE: Receive data register not empty (receivers) */
        mcu_reg_t addr           : 1; /*!< 3 ADDR: Address matched (slave mode) */
        mcu_reg_t nackf          : 1; /*!< 4 NACKF: Not Acknowledge received flag */
        mcu_reg_t stopf          : 1; /*!< 5 STOPF: Stop detection flag */
        mcu_reg_t tc             : 1; /*!< 6 TC: Trancfer Complete (master mode) */
        mcu_reg_t tcr            : 1; /*!< 7 TCR: Trancfer Complete Reload */
        mcu_reg_t berr           : 1; /*!< 8 BERR: Bus error */
        mcu_reg_t arlo           : 1; /*!< 9 ARLO: Arbitration lost */
        mcu_reg_t ovr            : 1; /*!< 10 OVR: Overrun/Underrun (slave mode) */
        mcu_reg_t pecerr         : 1; /*!< 11 PECERR: PEC Error in reception */
        mcu_reg_t timeout        : 1; /*!< 12 TIMEOUT: Timeout or tlow detection flag */
        mcu_reg_t alert          : 1; /*!< 13 ALERT: SMBus alert */
        mcu_reg_t Reserved14     : 1; /*!< 14 Reserved, must be kept at reset value.*/
        mcu_reg_t busy           : 1; /*!< 15 BUSY: Bus busy */
        mcu_reg_t dir            : 1; /*!< 16 DIR: Transfer direction (slave mode) */
        mcu_reg_t addcode        : 7; /*!< 23:17 ADDCODE[6:0] Address match code (slave mode) */
        mcu_reg_t Reserved31_24  : 8; /*!< 31:24 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C interrupt clear register (I2C_ICR)
 *
 *        Address offset: 0x1C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_icr_t,
    {
        mcu_reg_t Reserved2_0    : 3; /*!< 2:0 Reserved, must be kept at reset value.*/
        mcu_reg_t addrcf         : 1; /*!< 3 ADDRCF: Address matched flag clear */
        mcu_reg_t nackcf         : 1; /*!< 4 NACKCF: Not Acknowledge received flag clear */
        mcu_reg_t stopcf         : 1; /*!< 5 STOPCF: Stop detection flag clear */
        mcu_reg_t Reserved7_6    : 2; /*!< 7:6 Reserved, must be kept at reset value.*/
        mcu_reg_t berrcf         : 1; /*!< 8 BERR: Bus error flag clear */
        mcu_reg_t arlocf         : 1; /*!< 9 ARLOCF: Arbitration lost flag clear */
        mcu_reg_t ovrcf          : 1; /*!< 10 OVRCF: Overrun/Underrun flag clear */
        mcu_reg_t peccf          : 1; /*!< 11 PECCF: PEC Error flag clear */
        mcu_reg_t timeoutcf      : 1; /*!< 12 TIMEOUTCF: Timeout detection flag clear */
        mcu_reg_t alertcf        : 1; /*!< 13 ALERTCF: Alert flag clear */
        mcu_reg_t Reserved31_14  : 18; /*!< 31:14 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C PEC register (I2C_PECR)
 *
 *        Address offset: 0x20
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_pecr_t,
    {
        mcu_reg_t pec            : 8; /*!< 7:0 PEC[7:0] Packet error checking reister */
        mcu_reg_t Reserved31_8   : 24; /*!< 31:8 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C receive data register (I2C_RXDR)
 *
 *        Address offset: 0x24
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_rxdr_t,
    {
        mcu_reg_t rxdata         : 8; /*!< 7:0 RXDATA[7:0] 8-bit receive data */
        mcu_reg_t Reserved31_8   : 24; /*!< 31:8 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C transmit data register (I2C_TXDR)
 *
 *        Address offset: 0x28
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    i2c_txdr_t,
    {
        mcu_reg_t txdata         : 8; /*!< 7:0 TXDATA[7:0] 8-bit transmit data */
        mcu_reg_t Reserved31_8   : 24; /*!< 31:8 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief I2C register map
 */
typedef __packed struct
{
    volatile i2c_cr1_t      CR1;            /*!< 1 0x00 */
    volatile i2c_cr2_t      CR2;            /*!< 2 0x04 */
    volatile i2c_oar1_t     OAR1;           /*!< 3 0x08 */
    volatile i2c_oar2_t     OAR2;           /*!< 4 0x0C */
    volatile i2c_timing_t   TIMING;         /*!< 5 0x10 */
    volatile i2c_timeout_t  TIMEOUT;        /*!< 6 0x14 */
    volatile i2c_isr_t      ISR;            /*!< 7 0x18 */
    volatile i2c_icr_t      ICR;            /*!< 8 0x1C */
    volatile i2c_pecr_t     PECR;           /*!< 9 0x20 */
    volatile i2c_rxdr_t     RXDR;           /*!< 10 0x24 */
    volatile i2c_txdr_t     TXDR;           /*!< 11 0x28 */
} i2c_regs_t;

CHECK_REG_MAP(i2c_regs_t, 11);

#define I2C1_REGS_ADDR (i2c_regs_t*)I2C1_BASE
#define I2C2_REGS_ADDR (i2c_regs_t*)I2C2_BASE
#define I2C3_REGS_ADDR (i2c_regs_t*)I2C3_BASE

#endif  // STM32L475_I2C_REG_H
