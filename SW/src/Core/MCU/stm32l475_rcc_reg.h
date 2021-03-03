/**
 * @file stm32l475_rcc_reg.h
 *
 *       Reset and clock control registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_RCC_REG_H
#define STM32L475_RCC_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief Clock control register (RCC_CR)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0063
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    rcc_cr_t,
    {
        mcu_reg_t msion          : 1; /*!< 0 MSION: MSI clock enable */
        mcu_reg_t msirdy         : 1; /*!< 1 MSIRDY: MSI clock ready flag */
        mcu_reg_t msipllen       : 1; /*!< 2 MSIPLLEN: MSI clock PLL enable */
        mcu_reg_t msirgsel       : 1; /*!< 3 MSIRGSEL: MSI clock range selection */
        mcu_reg_t msirange       : 4; /*!< 7:4 MSIRANGE[3:0] MSI clock ranges */
        mcu_reg_t hsion          : 1; /*!< 8 HSION: HSI16 clock enable */
        mcu_reg_t hsikeron       : 1; /*!< 9 HSIKERON: HSI16 always enable for peripheral kernels */
        mcu_reg_t hsirdy         : 1; /*!< 10 HSIRDY: HSI16 clock enable */
        mcu_reg_t hsiasfs        : 1; /*!< 11 HSIASFS: HSI16 automatic start from Stop */
        mcu_reg_t Reserved12_15  : 4; /*!< 15:12 Reserved, must be kept at reset value */
        mcu_reg_t hseon          : 1; /*!< 16 HSEON: HSE clock enable */
        mcu_reg_t hserdy         : 1; /*!< 17 HSERDY: HSE clock ready flag */
        mcu_reg_t hsebyp         : 1; /*!< 18 HSEBYP: HSE clock bypass */
        mcu_reg_t csson          : 1; /*!< 19 CSSON: Clock security system enable */
        mcu_reg_t Reserved20_23  : 4; /*!< 23:20 Reserved, must be kept at reset value */
        mcu_reg_t pllon          : 1; /*!< 24 PLLON: Main PLL enable */
        mcu_reg_t pllrdy         : 1; /*!< 25 PLLRDY: Main PLL clock ready flag */
        mcu_reg_t pllsai1on      : 1; /*!< 26 PLLSAI1ON: SAI1 PLL enable */
        mcu_reg_t pllsai1rdy     : 1; /*!< 27 PLLSAI1RDY: SAI1 PLL clock ready flag */
        mcu_reg_t pllsai2on      : 1; /*!< 28 PLLSAI2ON: SAI2 PLL enable */
        mcu_reg_t pllsai2rdy     : 1; /*!< 29 PLLSAI2RDY: SAI2 PLL clock ready flag */
        mcu_reg_t Reserved30_31  : 2; /*!< 31:30 Reserved, must be kept at reset value */
    }
);


/**
 * @brief Internal clock sources calibration register (RCC_ICSCR)
 *
 *        Address offset: 0x04
 *        Reset value: 0x10XX 00XX where X is factory-programmed
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_icsc_t,
    {
        mcu_reg_t msical         : 8; /*!< 7:0 MSICAL[7:0] MSI clock calibration */
        mcu_reg_t msitrim        : 8; /*!< 15:8 MSITRIM[7:0] MSI clock trimming */
        mcu_reg_t hsical         : 8; /*!< 23:16 HSICAL[7:0] HSI16 clock calibration */
        mcu_reg_t hsitrim        : 7; /*!< 24:30 HSITRIM[6:0] HSI16 clock trimming */
        mcu_reg_t Reserved31     : 1; /*!< 31 Reserved, must be kept at reset value */
    }
);


/**
 * @brief Clock configuration register (RCC_CFGR)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 *        Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
 *        1 or 2 wait states inserted only if the access occurs during a clock source switch.
 *        From 0 to 15 wait states inserted if the access occurs the APB or AHB prescalers values update is on going.
 */
MCU_REG_TYPE (
    rcc_cfg_t,
    {
        mcu_reg_t sw             : 2; /*!< 1:0 SW[1:0]: System clock switch */
        mcu_reg_t sws            : 2; /*!< 3:2 SWS[1:0]: System clock switch status */
        mcu_reg_t hpre           : 4; /*!< 7:4 HPRE[3:0]: AHB prescaler */
        mcu_reg_t ppre1          : 3; /*!< 10:8 PPRE1[2:0]: APB Low speed prescaler (APB1) */
        mcu_reg_t ppre2          : 3; /*!< 13:11 PPRE2[2:0]: APB high-speed prescaler (APB2) */
        mcu_reg_t Reserved14     : 1; /*!< 14 Reserved, must be kept at reset value */
        mcu_reg_t stopwuck       : 1; /*!< 15 STOPWUCK: Wakeup from Stop and CSS backup clock selection */
        mcu_reg_t Reserved16_23  : 8; /*!< 23:16 Reserved, must be kept at reset value */
        mcu_reg_t mcosel         : 4; /*!< 27:24 MCOSEL[3:0]: Microcontroller clock output */
        mcu_reg_t mcopre         : 3; /*!< 30:28 MCOPRE[2:0]: Microcontroller clock output prescaler */
        mcu_reg_t Reserved31     : 1; /*!< 31 Reserved, must be kept at reset value */
    }
);

/**
 * @brief PLL configuration register (RCC_PLLCFGR)
 *
 *        Address offset: 0x0C
 *        Reset value: 0x0000 1000
 *        Access: no wait state, word, half-word and byte access.
 *
 *        This register is used to configure the PLL clock outputs according
 *        to the formulas:
 *        * f(VCOclock) = f(PLL clock input) * (PLLN / PLLM)
 *        * f(PLL_P) = f(VCO clock) / PLLP
 *        * f(PLL_Q) = f(VCO clock) / PLLQ
 *        * f(PLL_R) = f(VCO clock) / PLLR
 */
MCU_REG_TYPE (
    rcc_pllcfg_t,
    {
        mcu_reg_t pllsrc         : 2; /*!< 1:0 PLLSRC[1:0]: Main PLL, PLLSAI1 and PLLSAI2 entry clock source */
        mcu_reg_t Reserved2_3    : 2; /*!< 3:2 Reserved, must be kept at reset value */
        mcu_reg_t pllm           : 3; /*!< 6:4 PLLM[2:0]: Division factor for the main PLL and audio PLL (PLLSAI1 and PLLSAI2) input clock */
        mcu_reg_t Reserved7      : 1; /*!< 7 Reserved, must be kept at reset value */
        mcu_reg_t plln           : 7; /*!< 14:8 PLLN[6:0]: Main PLL multiplication factor for VCO */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t pllpen         : 1; /*!< 16 PLLPEN: Main PLL, PLLSAI3CLK output enable */
        mcu_reg_t pllp           : 1; /*!< 17 PLLP: Main PLL division factor for PLLSAI3CLK (SAI1 and SAI2 clock) */
        mcu_reg_t Reserved18_19  : 2; /*!< 19:18 Reserved, must be kept at reset value */
        mcu_reg_t pllqen         : 1; /*!< 20 PLLQEN: Main PLL PLL48M1CLK output enable */
        mcu_reg_t pllq           : 2; /*!< 22:21 PLLQ[1:0]: Main PLL division factor for PLL48M1CLK (48 MHz clock) */
        mcu_reg_t Reserved23     : 1; /*!< 23 Reserved, must be kept at reset value */
        mcu_reg_t pllren         : 1; /*!< 24 PLLREN: Main PLL PLLCLK output enable */
        mcu_reg_t pllr           : 2; /*!< 26:25 PLLR[1:0]: Main PLL division factor for PLLCLK (system clock) */
        mcu_reg_t pllpdiv        : 5; /*!< 31:27 PLLPDIV[4:0]: Main PLL division factor for PLLSAI2CLK */
    }
);

/**
 * @brief PLLSAI1 configuration register (RCC_PLLSAI1CFGR)
 *
 *        Address offset: 0x10
 *        Reset value: 0x0000 1000
 *        Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
 *        1 or 2 wait states inserted only if the access occurs during a clock source switch.
 *        This register is used to configure the PLLSAI1 clock outputs according
 *        to the formulas:
 *        * f(VCOSAI1 clock) = f(PLL clock input) * (PLLSAI1N / PLLM)
 *        * f(PLLSAI1_P) = f(VCOSAI1 clock) / PLLSAI1P
 *        * f(PLLSAI1_Q) = f(VCOSAI1 clock) / PLLSAI1Q
 *        * f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
 */
MCU_REG_TYPE (
    rcc_pllsai1cfg_t,
    {
        mcu_reg_t Reserved0_7    : 8; /*!< 7:0 Reserved, must be kept at reset value */
        mcu_reg_t pllsai1n       : 7; /*!< 14:8 PLLSAI1[6:0] multiplication factor for VCO */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t pllsai1pen     : 1; /*!< 16 PLLSAI1PEN: PLLSAI1 PLLSAI1CLK output enable */
        mcu_reg_t pllsai1p       : 1; /*!< 17 PLLSAI1P: PLLSAI1 division factor for PLLSAI1CLK (SAI1 or SAI2 clock) */
        mcu_reg_t Reserved18_19  : 2; /*!< 19:18 Reserved, must be kept at reset value */
        mcu_reg_t pllsai1qen     : 1; /*!< 20 PLLSAI1QEN: PLLSAI1 PLL48M2CLK output enable */
        mcu_reg_t pllsai1q       : 2; /*!< 22:21 PLLSAI1Q[1:0] PLLSAI1 division factor for PLL48M2CLK (48 MHz clock) */
        mcu_reg_t Reserved23     : 1; /*!< 23 Reserved, must be kept at reset value */
        mcu_reg_t pllsai1ren     : 1; /*!< 24 PLLSAI1REN: PLLSAI1 PLLADC1CLK output enable */
        mcu_reg_t pllsai1r       : 2; /*!< 26:25 PLLSAI1R[1:0] PLLSAI1 division factor for PLLADC1CLK (ADC clock)*/
        mcu_reg_t pllsai1pdiv    : 5; /*!< 31:27 PLLSAI1PDIV[4:0] PLLSAI1 division factor for PLLSAI1CLK */
    }
);

/**
 * @brief PLLSAI2 configuration register (RCC_PLLSAI2CFGR)
 *
 *        Address offset: 0x14
 *        Reset value: 0x0000 1000
 *        Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
 *        1 or 2 wait states inserted only if the access occurs during a clock source switch.
 *        This register is used to configure the PLLSAI2 clock outputs according
 *        to the formulas:
 *        * f(VCOSAI2 clock) = f(PLL clock input) * (PLLSAI2N / PLLM)
 *        * f(PLLSAI2_P) = f(VCOSAI2 clock) / PLLSAI2P
 *        * f(PLLSAI2_R) = f(VCOSAI2 clock) / PLLSAI2R
 */
MCU_REG_TYPE (
    rcc_pllsai2cfg_t,
    {
        mcu_reg_t Reserved0_7    : 8; /*!< 7:0 Reserved, must be kept at reset value */
        mcu_reg_t pllsai2n       : 7; /*!< 14:8 PLLSAI2[6:0] multiplication factor for VCO */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t pllsai2pen     : 1; /*!< 16 PLLSAI2PEN: PLLSAI2 PLLSAI2CLK output enable */
        mcu_reg_t pllsai2p       : 1; /*!< 17 PLLSAI2P: PLLSAI2 division factor for PLLSAI2CLK (SAI1 or SAI2 clock) */
        mcu_reg_t Reserved18_23  : 6; /*!< 23:18 Reserved, must be kept at reset value */
        mcu_reg_t pllsai2ren     : 1; /*!< 24 PLLSAI2REN: PLLSAI2 PLLADC2CLK output enable */
        mcu_reg_t pllsai2r       : 2; /*!< 26:25 PLLSAI2R[1:0] PLLSAI2 division factor for PLLADC2CLK (ADC clock)*/
        mcu_reg_t pllsai2pdiv    : 5; /*!< 31:27 PLLSAI2PDIV[4:0] PLLSAI2 division factor for PLLSAI2CLK */
    }
);

/**
 * @brief Interrupt enable register (RCC_CIER)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    rcc_cie_t,
    {
        mcu_reg_t lsirdyie       : 1; /*!< 0 LSIRDYIE: LSI ready interrupt enable */
        mcu_reg_t lserdyie       : 1; /*!< 1 LSERDYIE: LSE ready interrupt enable */
        mcu_reg_t msirdyie       : 1; /*!< 2 MSIRDYIE: MSI ready interrupt enable */
        mcu_reg_t hsirdyie       : 1; /*!< 3 HSIRDYIE: HSI16 ready interrupt enable */
        mcu_reg_t hserdyie       : 1; /*!< 4 HSERDYIE: HSE ready interrupt enable */
        mcu_reg_t pllrdyie       : 1; /*!< 5 PLLRDYIE: PLL ready interrupt enable */
        mcu_reg_t pllsai1rdyie   : 1; /*!< 6 PLLSAI1RDYIE: PLLSAI1 ready interrupt enable */
        mcu_reg_t pllsai2rdyie   : 1; /*!< 7 PLLSAI2RDYIE: PLLSAI2 ready interrupt enable */
        mcu_reg_t Reserved8      : 1; /*!< 8 Reserved, must be kept at reset value */
        mcu_reg_t lsecssie       : 1; /*!< 9 LSECSSIE: LSE clock security interrupt enable */
        mcu_reg_t hsi48rdyie     : 1; /*!< 10 HSI48RDYIE: HSI48 ready interrupt enable */
        mcu_reg_t Reserved11_31  : 21; /*!< 31:11 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Clock interrupt flag register (RCC_CIFR)
 *
 *        Address offset: 0x1C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    rcc_cif_t,
    {
        mcu_reg_t lsirdy         : 1; /*!< 0 LSIRDYF: LSI ready interrupt flag */
        mcu_reg_t lserdy         : 1; /*!< 1 LSERDYF: LSE ready interrupt flag */
        mcu_reg_t msirdy         : 1; /*!< 2 MSIRDYF: MSI ready interrupt flag */
        mcu_reg_t hsirdy         : 1; /*!< 3 HSIRDYF: HSI16 ready interrupt flag */
        mcu_reg_t hserdy         : 1; /*!< 4 HSERDYF: HSE ready interrupt flag */
        mcu_reg_t pllrdy         : 1; /*!< 5 PLLRDYF: PLL ready interrupt flag */
        mcu_reg_t pllsai1rdy     : 1; /*!< 6 PLLSAI1RDYF: PLLSAI1 ready interrupt flag */
        mcu_reg_t pllsai2rdy     : 1; /*!< 7 PLLSAI2RDYF: PLLSAI2 ready interrupt flag */
        mcu_reg_t css            : 1; /*!< 8 CSSF: Clock security system interrupt flag */
        mcu_reg_t lsecss         : 1; /*!< 9 LSECSSF: Clock security system interrupt flag */
        mcu_reg_t hsi48rdy       : 1; /*!< 10 HSI48RDYF: HSI48 ready interrupt flag */
        mcu_reg_t Reserved11_31  : 21; /*!< 31:11 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Clock interrupt clear register (RCC_CICR)
 *
 *        Address offset: 0x20
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    rcc_cic_t,
    {
        mcu_reg_t lsirdyc        : 1; /*!< 0 LSIRDYC: LSI ready interrupt clear */
        mcu_reg_t lserdyc        : 1; /*!< 1 LSERDYC: LSE ready interrupt clear */
        mcu_reg_t msirdyc        : 1; /*!< 2 MSIRDYC: MSI ready interrupt clear */
        mcu_reg_t hsirdyc        : 1; /*!< 3 HSIRDYC: HSI16 ready interrupt clear */
        mcu_reg_t hserdyc        : 1; /*!< 4 HSERDYC: HSE ready interrupt clear */
        mcu_reg_t pllrdyc        : 1; /*!< 5 PLLRDYC: PLL ready interrupt clear */
        mcu_reg_t pllsai1rdyc    : 1; /*!< 6 PLLSAI1RDYC: PLLSAI1 ready interrupt clear */
        mcu_reg_t pllsai2rdyc    : 1; /*!< 7 PLLSAI2RDYC: PLLSAI2 ready interrupt clear */
        mcu_reg_t cssc           : 1; /*!< 8 CSSC: Clock security system interrupt clear */
        mcu_reg_t lsecssc        : 1; /*!< 9 LSECSSC: Clock security system interrupt clear */
        mcu_reg_t hsi48rdyc      : 1; /*!< 10 HSI48RDYC: HSI48 ready interrupt clear */
        mcu_reg_t Reserved11_31  : 21; /*!< 31:11 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB1 peripheral reset register (RCC_AHB1RSTR)
 *
 *        Address offset: 0x28
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb1rst_t,
    {
        mcu_reg_t dma1rst        : 1; /*!< 0 DMA1RST: DMA1 reset */
        mcu_reg_t dma2rst        : 1; /*!< 1 DMA2RST: DMA2 reset */
        mcu_reg_t Reserved2_7    : 6; /*!< 7:2 Reserved, must be kept at reset value */
        mcu_reg_t flashrst       : 1; /*!< 8 FLASHRST: Flash memory interface reset */
        mcu_reg_t Reserved9_11   : 3; /*!< 11:9 Reserved, must be kept at reset value */
        mcu_reg_t crcrst         : 1; /*!< 12 CRCRST: CRC reset */
        mcu_reg_t Reserved13_15  : 3; /*!< 15:13 Reserved, must be kept at reset value */
        mcu_reg_t tscrst         : 1; /*!< 16 TSCRST: Touch Sensing Controller reset */
        mcu_reg_t dma2drst       : 1; /*!< 17 DMA2DRST: DMA2D reset */
        mcu_reg_t Reserved18_31  : 14; /*!< 31:18 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB2 peripheral reset register (RCC_AHB2RSTR)
 *
 *        Address offset: 0x2C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb2rst_t,
    {
        mcu_reg_t gpioarst       : 1; /*!< 0 GPIOARST: IO port A reset */
        mcu_reg_t gpiobrst       : 1; /*!< 1 GPIOBRST: IO port B reset */
        mcu_reg_t gpiocrst       : 1; /*!< 2 GPIOCRST: IO port C reset */
        mcu_reg_t gpiodrst       : 1; /*!< 3 GPIODRST: IO port D reset */
        mcu_reg_t gpioerst       : 1; /*!< 4 GPIOERST: IO port E reset */
        mcu_reg_t gpiofrst       : 1; /*!< 5 GPIOFRST: IO port F reset */
        mcu_reg_t gpiogrst       : 1; /*!< 6 GPIOGRST: IO port G reset */
        mcu_reg_t gpiohrst       : 1; /*!< 7 GPIOHRST: IO port H reset */
        mcu_reg_t gpioirst       : 1; /*!< 8 GPIOFRST: IO port I reset */
        mcu_reg_t Reserved9_11   : 3; /*!< 11:9 Reserved, must be kept at reset value */
        mcu_reg_t otgfsrst       : 1; /*!< 12 OTGFSRST: USB OTG FS reset */
        mcu_reg_t adcrst         : 1; /*!< 13 ADCRST: ADC reset */
        mcu_reg_t dcmirst        : 1; /*!< 14 DCMIRST: Digital Camera interface reset */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t aesrst         : 1; /*!< 16 AESRST: AES hardware accelerator reset */
        mcu_reg_t hashrst        : 1; /*!< 17 HASHRST: HASH reset */
        mcu_reg_t rngrst         : 1; /*!< 18 RNGRST: Random number generator reset */
        mcu_reg_t Reserved19_31  : 13; /*!< 31:19 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB3 peripheral reset register (RCC_AHB3RSTR)
 *
 *        Address offset: 0x30
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb3rst_t,
    {
        mcu_reg_t fmcrst         : 1; /*!< 0 FMCRST: Flexible memory controller reset */
        mcu_reg_t Reserved1_7    : 7; /*!< 7:1 Reserved, must be kept at reset value */
        mcu_reg_t qspirst        : 1; /*!< 8 QSPIRST: QUADSPI1 memory interface reset */
        mcu_reg_t Reserved9_31   : 23; /*!< 31:9 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB1 peripheral reset register 1 (RCC_APB1RSTR1)
 *
 *        Address offset: 0x38
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb1rst1_t,
    {
        mcu_reg_t tim2rst        : 1; /*!< 0 TIM2RST: TIM2 timer reset */
        mcu_reg_t tim3rst        : 1; /*!< 1 TIM3RST: TIM3 timer reset */
        mcu_reg_t tim4rst        : 1; /*!< 2 TIM4RST: TIM4 timer reset */
        mcu_reg_t tim5rst        : 1; /*!< 3 TIM5RST: TIM5 timer reset */
        mcu_reg_t tim6rst        : 1; /*!< 4 TIM6RST: TIM6 timer reset */
        mcu_reg_t tim7rst        : 1; /*!< 5 TIM7RST: TIM7 timer reset */
        mcu_reg_t Reserved6_8    : 3; /*!< 8:6 Reserved, must be kept at reset value */
        mcu_reg_t lcdrst         : 1; /*!< 9 LCDRST: LCD interface reset */
        mcu_reg_t Reserved10_13  : 4; /*!< 13:10 Reserved, must be kept at reset value */
        mcu_reg_t spi2rst        : 1; /*!< 14 SPI2RST: SPI 2 reset */
        mcu_reg_t spi3rst        : 1; /*!< 15 SPI3RST: SPI 3 reset */
        mcu_reg_t Reserved16     : 1; /*!< 16 Reserved, must be kept at reset value */
        mcu_reg_t usart2rst      : 1; /*!< 17 USART2RST: USART 2 reset */
        mcu_reg_t usart3rst      : 1; /*!< 18 USART3RST: USART 3 reset */
        mcu_reg_t usart4rst      : 1; /*!< 19 USART4RST: USART 4 reset */
        mcu_reg_t usart5rst      : 1; /*!< 20 USART5RST: USART 5 reset */
        mcu_reg_t i2c1rst        : 1; /*!< 21 I2C1RST: I2C 1 reset */
        mcu_reg_t i2c2rst        : 1; /*!< 22 I2C2RST: I2C 2 reset */
        mcu_reg_t i2c3rst        : 1; /*!< 23 I2C3RST: I2C 3 reset */
        mcu_reg_t crsrst         : 1; /*!< 24 CRSRST: CRS reset */
        mcu_reg_t can1rst        : 1; /*!< 25 CAN1RST: CAN1 reset */
        mcu_reg_t can2rst        : 1; /*!< 26 CAN2RST: CAN2 reset */
        mcu_reg_t Reserved27     : 1; /*!< 27 Reserved, must be kept at reset value */
        mcu_reg_t pwrrst         : 1; /*!< 28 PWRRST: Power interface reset */
        mcu_reg_t dac1rst        : 1; /*!< 29 DAC1RST: DAC1 interface reset */
        mcu_reg_t opamprst       : 1; /*!< 30 OPAMPRST: OPAMP interface reset */
        mcu_reg_t lptim1rst      : 1; /*!< 31 LPTIM1RST: Low Power Timer 1 reset */
    }
);

/**
 * @brief APB1 peripheral reset register 2 (RCC_APB1RSTR2)
 *
 *        Address offset: 0x3C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb1rst2_t,
    {
        mcu_reg_t lpuart1rst     : 1; /*!< 0 LPUART1RST: Low-power UART 1 reset */
        mcu_reg_t i2c4rst        : 1; /*!< 1 I2C4RST: I2C 4 reset */
        mcu_reg_t swpmi1rst      : 1; /*!< 2 SWPMI1RST: Single wire protocol reset */
        mcu_reg_t Reserved3_4    : 2; /*!< 4:3 Reserved, must be kept at reset value */
        mcu_reg_t lptim2rst      : 1; /*!< 5 LPTIM2RST: Low Power Timer 2 reset */
        mcu_reg_t Reserved6_31   : 26; /*!< 31:6 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB2 peripheral reset register (RCC_APB2RSTR)
 *
 *        Address offset: 0x40
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb2rst_t,
    {
        mcu_reg_t syscfgrst      : 1; /*!< 0 SYSCFGRST: SYSCFG + COMP + VREFBUF reset */
        mcu_reg_t Reserved1_9    : 9; /*!< 9:1 Reserved, must be kept at reset value */
        mcu_reg_t sdmmc1rst      : 1; /*!< 10 SDMMC1RST: SDMMC reset */
        mcu_reg_t tim1rst        : 1; /*!< 11 TIM1RST: TIM1 timer reset */
        mcu_reg_t spi1rst        : 1; /*!< 12 SPI1RST: SPI 1 reset */
        mcu_reg_t tim8rst        : 1; /*!< 13 TIM8RST: TIM8 timer reset */
        mcu_reg_t usart1rst      : 1; /*!< 14 USART1RST: USART 1 reset */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t tim15rst       : 1; /*!< 16 TIM15RST: TIM15 timer reset */
        mcu_reg_t tim16rst       : 1; /*!< 17 TIM16RST: TIM16 timer reset */
        mcu_reg_t tim17rst       : 1; /*!< 18 TIM17RST: TIM17 timer reset */
        mcu_reg_t Reserved19_20  : 2; /*!< 20:19 Reserved, must be kept at reset value */
        mcu_reg_t sai1rst        : 1; /*!< 21 SAI1RST: Serial audio interface 1 (SAI1) reset */
        mcu_reg_t sai2rst        : 1; /*!< 22 SAI2RST: Serial audio interface 2 (SAI2) reset */
        mcu_reg_t Reserved23     : 1; /*!< 23 Reserved, must be kept at reset value */
        mcu_reg_t dfsdm1rst      : 1; /*!< 24 DFSDM1RST: Digital filters for sigma-delta modulators (DFSDM1) reset */
        mcu_reg_t Reserved25_31  : 7; /*!< 31:25 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB1 peripheral clock enable register (RCC_AHB1ENR)
 *
 *        Address offset: 0x48
 *        Reset value: 0x0000 0100
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_ahb1en_t,
    {
        mcu_reg_t dma1en         : 1; /*!< 0 DMA1EN: DMA1 clock enable */
        mcu_reg_t dma2en         : 1; /*!< 1 DMA2EN: DMA2 clock enable */
        mcu_reg_t Reserved2_7    : 6; /*!< 7:2 Reserved, must be kept at reset value */
        mcu_reg_t flashen        : 1; /*!< 8 FLASHEN: Flash memory interface clock enable */
        mcu_reg_t Reserved9_11   : 3; /*!< 11:9 Reserved, must be kept at reset value */
        mcu_reg_t crcen          : 1; /*!< 12 CRCEN: CRC clock enable */
        mcu_reg_t Reserved13_15  : 3; /*!< 15:13 Reserved, must be kept at reset value */
        mcu_reg_t tscen          : 1; /*!< 16 TSCEN: Touch Sensing Controller clock enable */
        mcu_reg_t dma2den        : 1; /*!< 17 DMA2DEN: DMA2D clock enable */
        mcu_reg_t Reserved18_31  : 14; /*!< 31:18 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB2 peripheral clock enable register (RCC_AHB2ENR)
 *
 *        Address offset: 0x4C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_ahb2en_t,
    {
        mcu_reg_t gpioaen        : 1; /*!< 0 GPIOAEN: IO port A clock enable */
        mcu_reg_t gpioben        : 1; /*!< 1 GPIOBEN: IO port B clock enable */
        mcu_reg_t gpiocen        : 1; /*!< 2 GPIOCEN: IO port C clock enable */
        mcu_reg_t gpioden        : 1; /*!< 3 GPIODEN: IO port D clock enable */
        mcu_reg_t gpioeen        : 1; /*!< 4 GPIOEEN: IO port E clock enable */
        mcu_reg_t gpiofen        : 1; /*!< 5 GPIOFEN: IO port F clock enable */
        mcu_reg_t gpiogen        : 1; /*!< 6 GPIOGEN: IO port G clock enable */
        mcu_reg_t gpiohen        : 1; /*!< 7 GPIOHEN: IO port H clock enable */
        mcu_reg_t gpioien        : 1; /*!< 8 GPIOFEN: IO port I clock enable */
        mcu_reg_t Reserved9_11   : 3; /*!< 11:9 Reserved, must be kept at reset value */
        mcu_reg_t otgfsen        : 1; /*!< 12 OTGFSEN: USB OTG FS clock enable */
        mcu_reg_t adcen          : 1; /*!< 13 ADCEN: ADC clock enable */
        mcu_reg_t dcmien         : 1; /*!< 14 DCMIEN: Digital Camera interface clock enable */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t aesen          : 1; /*!< 16 AESEN: AES hardware accelerator clock enable */
        mcu_reg_t hashen         : 1; /*!< 17 HASHEN: HASH clock enable */
        mcu_reg_t rngen          : 1; /*!< 18 RNGEN: Random number generator clock enable */
        mcu_reg_t Reserved19_31  : 13; /*!< 31:19 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB3 peripheral clock enable register (RCC_AHB3ENR)
 *
 *        Address offset: 0x50
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_ahb3en_t,
    {
        mcu_reg_t fmcen          : 1; /*!< 0 FMCEN: Flexible memory controller clock enable */
        mcu_reg_t Reserved1_7    : 7; /*!< 7:1 Reserved, must be kept at reset value */
        mcu_reg_t qspien         : 1; /*!< 8 QSPIEN: QUADSPI1 memory interface clock enable */
        mcu_reg_t Reserved9_31   : 23; /*!< 31:9 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB1 peripheral clock enable register 1 (RCC_APB1ENR1)
 *
 *        Address offset: 0x58
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_apb1en1_t,
    {
        mcu_reg_t tim2en         : 1; /*!< 0 TIM2EN: TIM2 timer clock enable */
        mcu_reg_t tim3en         : 1; /*!< 1 TIM3EN: TIM3 timer clock enable */
        mcu_reg_t tim4en         : 1; /*!< 2 TIM4EN: TIM4 timer clock enable */
        mcu_reg_t tim5en         : 1; /*!< 3 TIM5EN: TIM5 timer clock enable */
        mcu_reg_t tim6en         : 1; /*!< 4 TIM6EN: TIM6 timer clock enable */
        mcu_reg_t tim7en         : 1; /*!< 5 TIM7EN: TIM7 timer clock enable */
        mcu_reg_t Reserved6_8    : 3; /*!< 8:6 Reserved, must be kept at reset value */
        mcu_reg_t lcden          : 1; /*!< 9 LCDEN: LCD interface clock enable */
        mcu_reg_t rtcapben       : 1; /*!< 10 RTCAPBEN: RTC APB clock enable */
        mcu_reg_t wwdgen         : 1; /*!< 11 WWDGEN: Window watchdog clock enable */
        mcu_reg_t Reserved12_13  : 2; /*!< 13:12 Reserved, must be kept at reset value */
        mcu_reg_t spi2en         : 1; /*!< 14 SPI2EN: SPI 2 clock enable */
        mcu_reg_t spi3en         : 1; /*!< 15 SPI3EN: SPI 3 clock enable */
        mcu_reg_t Reserved16     : 1; /*!< 16 Reserved, must be kept at reset value */
        mcu_reg_t usart2en       : 1; /*!< 17 USART2EN: USART 2 clock enable */
        mcu_reg_t usart3en       : 1; /*!< 18 USART3EN: USART 3 clock enable */
        mcu_reg_t usart4en       : 1; /*!< 19 USART4EN: USART 4 clock enable */
        mcu_reg_t usart5en       : 1; /*!< 20 USART5EN: USART 5 clock enable */
        mcu_reg_t i2c1en         : 1; /*!< 21 I2C1EN: I2C 1 clock enable */
        mcu_reg_t i2c2en         : 1; /*!< 22 I2C2EN: I2C 2 clock enable */
        mcu_reg_t i2c3en         : 1; /*!< 23 I2C3EN: I2C 3 clock enable */
        mcu_reg_t crsen          : 1; /*!< 24 CRSEN: CRS clock enable */
        mcu_reg_t can1en         : 1; /*!< 25 CAN1EN: CAN1 clock enable */
        mcu_reg_t can2en         : 1; /*!< 26 CAN2EN: CAN2 clock enable */
        mcu_reg_t Reserved27     : 1; /*!< 27 Reserved, must be kept at reset value */
        mcu_reg_t pwren          : 1; /*!< 28 PWREN: Power interface clock enable */
        mcu_reg_t dac1en         : 1; /*!< 29 DAC1EN: DAC1 interface clock enable */
        mcu_reg_t opampen        : 1; /*!< 30 OPAMPEN: OPAMP interface clock enable */
        mcu_reg_t lptim1en       : 1; /*!< 31 LPTIM1EN: Low Power Timer 1 clock enable */
    }
);

/**
 * @brief APB1 peripheral clock enable register 2 (RCC_APB1ENR2)
 *
 *        Address offset: 0x5C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_apb1en2_t,
    {
        mcu_reg_t lpuart1en      : 1; /*!< 0 LPUART1EN: Low-power UART 1 clock enable */
        mcu_reg_t i2c4en         : 1; /*!< 1 I2C4EN: I2C 4 clock enable */
        mcu_reg_t swpmi1en       : 1; /*!< 2 SWPMI1EN: Single wire protocol clock enable */
        mcu_reg_t Reserved3_4    : 2; /*!< 4:3 Reserved, must be kept at reset value */
        mcu_reg_t lptim2en       : 1; /*!< 5 LPTIM2EN: Low Power Timer 2 clock enable */
        mcu_reg_t Reserved6_31   : 26; /*!< 31:6 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB2 peripheral clock enable register (RCC_APB2ENR)
 *
 *        Address offset: 0x60
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access.
 *        When the peripheral clock in not active, the peripheral registers read or write access is not supprted.
 */
MCU_REG_TYPE (
    rcc_apb2en_t,
    {
        mcu_reg_t syscfgen       : 1; /*!< 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable */
        mcu_reg_t Reserved1_6    : 6; /*!< 6:1 Reserved, must be kept at reset value */
        mcu_reg_t fwen           : 1; /*!< 7 FWEN: Firewall clock enable */
        mcu_reg_t Reserved8_9    : 2; /*!< 9:8 Reserved, must be kept at reset value */
        mcu_reg_t sdmmc1en       : 1; /*!< 10 SDMMC1EN: SDMMC clock enable */
        mcu_reg_t tim1en         : 1; /*!< 11 TIM1EN: TIM1 timer clock enable */
        mcu_reg_t spi1en         : 1; /*!< 12 SPI1EN: SPI 1 clock enable */
        mcu_reg_t tim8en         : 1; /*!< 13 TIM8EN: TIM8 timer clock enable */
        mcu_reg_t usart1en       : 1; /*!< 14 USART1EN: USART 1 clock enable */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t tim15en        : 1; /*!< 16 TIM15EN: TIM15 timer clock enable */
        mcu_reg_t tim16en        : 1; /*!< 17 TIM16EN: TIM16 timer clock enable */
        mcu_reg_t tim17en        : 1; /*!< 18 TIM17EN: TIM17 timer clock enable */
        mcu_reg_t Reserved19_20  : 2; /*!< 20:19 Reserved, must be kept at reset value */
        mcu_reg_t sai1en         : 1; /*!< 21 SAI1EN: Serial audio interface 1 (SAI1) clock enable */
        mcu_reg_t sai2en         : 1; /*!< 22 SAI2EN: Serial audio interface 2 (SAI2) clock enable */
        mcu_reg_t Reserved23     : 1; /*!< 23 Reserved, must be kept at reset value */
        mcu_reg_t dfsdm1en       : 1; /*!< 24 DFSDM1EN: Digital filters for sigma-delta modulators (DFSDM1) clock enable */
        mcu_reg_t Reserved25_31  : 7; /*!< 31:25 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB1 peripheral clock enable in Sleep and Stop modes register (RCC_AHB1SMENR)
 *
 *        Address offset: 0x68
 *        Reset value: 0x0001 1303
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb1smen_t,
    {
        mcu_reg_t dma1smen       : 1; /*!< 0 DMA1SMEN: DMA1 clock enable in Sleep and Stop modes */
        mcu_reg_t dma2smen       : 1; /*!< 1 DMA2SMEN: DMA2 clock enable in Sleep and Stop modes */
        mcu_reg_t Reserved2_7    : 6; /*!< 7:2 Reserved, must be kept at reset value */
        mcu_reg_t flashsmen      : 1; /*!< 8 FLASHSMEN: Flash memory interface clock enable in Sleep and Stop modes */
        mcu_reg_t sram1smen      : 1; /*!< 9 SRAM1SMEN: SRAM1 interface clock enable in Sleep and Stop modes */
        mcu_reg_t Reserved10_11  : 2; /*!< 11:9 Reserved, must be kept at reset value */
        mcu_reg_t crcsmen        : 1; /*!< 12 CRCSMEN: CRC clock enable in Sleep and Stop modes */
        mcu_reg_t Reserved13_15  : 3; /*!< 15:13 Reserved, must be kept at reset value */
        mcu_reg_t tscsmen        : 1; /*!< 16 TSCSMEN: Touch Sensing Controller clock enable in Sleep and Stop modes */
        mcu_reg_t dma2dsmen      : 1; /*!< 17 DMA2DSMEN: DMA2D clock enable in Sleep and Stop modes */
        mcu_reg_t Reserved18_31  : 14; /*!< 31:18 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB2 peripheral clock enable in Sleep and Stop modes register (RCC_AHB2SMENR)
 *
 *        Address offset: 0x6C
 *        Reset value: 0x0005 32FF
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb2smen_t,
    {
        mcu_reg_t gpioasmen      : 1; /*!< 0 GPIOASMEN: IO port A clock enable during Sleep and Stop modes */
        mcu_reg_t gpiobsmen      : 1; /*!< 1 GPIOBSMEN: IO port B clock enable during Sleep and Stop modes */
        mcu_reg_t gpiocsmen      : 1; /*!< 2 GPIOCSMEN: IO port C clock enable during Sleep and Stop modes */
        mcu_reg_t gpiodsmen      : 1; /*!< 3 GPIODSMEN: IO port D clock enable during Sleep and Stop modes */
        mcu_reg_t gpioesmen      : 1; /*!< 4 GPIOESMEN: IO port E clock enable during Sleep and Stop modes */
        mcu_reg_t gpiofsmen      : 1; /*!< 5 GPIOFSMEN: IO port F clock enable during Sleep and Stop modes */
        mcu_reg_t gpiogsmen      : 1; /*!< 6 GPIOGSMEN: IO port G clock enable during Sleep and Stop modes */
        mcu_reg_t gpiohsmen      : 1; /*!< 7 GPIOHSMEN: IO port H clock enable during Sleep and Stop modes */
        mcu_reg_t gpioismen      : 1; /*!< 8 GPIOFSMEN: IO port I clock enable during Sleep and Stop modes */
        mcu_reg_t sram2smen      : 1; /*!< 9 SRAM2SMEN: SRAM2 interface clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved10_11  : 2; /*!< 11:10 Reserved, must be kept at reset value */
        mcu_reg_t otgfssmen      : 1; /*!< 12 OTGFSSMEN: USB OTG FS clock enable during Sleep and Stop modes */
        mcu_reg_t adcsmen        : 1; /*!< 13 ADCSMEN: ADC clock enable during Sleep and Stop modes */
        mcu_reg_t dcmismen       : 1; /*!< 14 DCMISMEN: Digital Camera interface clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t aessmen        : 1; /*!< 16 AESSMEN: AES hardware accelerator clock enable during Sleep and Stop modes */
        mcu_reg_t hashsmen       : 1; /*!< 17 HASHSMEN: HASH clock enable during Sleep and Stop modes */
        mcu_reg_t rngsmen        : 1; /*!< 18 RNGSMEN: Random number gsmenerator clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved19_31  : 13; /*!< 31:19 Reserved, must be kept at reset value */
    }
);

/**
 * @brief AHB3 peripheral clock enable in Sleep and Stop modes register (RCC_AHB3SMENR)
 *
 *        Address offset: 0x70
 *        Reset value: 0x0000 0101
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_ahb3smen_t,
    {
        mcu_reg_t fmcsmen        : 1; /*!< 0 FMCSMEN: Flexible memory controller clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved1_7    : 7; /*!< 7:1 Reserved, must be kept at reset value */
        mcu_reg_t qspismen       : 1; /*!< 8 QSPISMEN: QUADSPI1 memory interface clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved9_31   : 23; /*!< 31:9 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB1 peripheral clock enable in Sleep and Stop modes register 1 (RCC_APB1SMENR1)
 *
 *        Address offset: 0x78
 *        Reset value: 0xF2FE CA3F
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb1smen1_t,
    {
        mcu_reg_t tim2smen       : 1; /*!< 0 TIM2SMEN: TIM2 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim3smen       : 1; /*!< 1 TIM3SMEN: TIM3 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim4smen       : 1; /*!< 2 TIM4SMEN: TIM4 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim5smen       : 1; /*!< 3 TIM5SMEN: TIM5 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim6smen       : 1; /*!< 4 TIM6SMEN: TIM6 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim7smen       : 1; /*!< 5 TIM7SMEN: TIM7 timer clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved6_8    : 3; /*!< 8:6 Reserved, must be kept at reset value */
        mcu_reg_t lcdsmen        : 1; /*!< 9 LCDSMEN: LCD interface clock enable during Sleep and Stop modes */
        mcu_reg_t rtcapbsmen     : 1; /*!< 10 RTCAPBSMEN: RTC APB clock enable during Sleep and Stop modes */
        mcu_reg_t wwdgsmen       : 1; /*!< 11 WWDGSMEN: Window watchdog clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved12_13  : 2; /*!< 13:12 Reserved, must be kept at reset value */
        mcu_reg_t spi2smen       : 1; /*!< 14 SPI2SMEN: SPI 2 clock enable during Sleep and Stop modes */
        mcu_reg_t spi3smen       : 1; /*!< 15 SPI3SMEN: SPI 3 clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved16     : 1; /*!< 16 Reserved, must be kept at reset value */
        mcu_reg_t usart2smen     : 1; /*!< 17 USART2SMEN: USART 2 clock enable during Sleep and Stop modes */
        mcu_reg_t usart3smen     : 1; /*!< 18 USART3SMEN: USART 3 clock enable during Sleep and Stop modes */
        mcu_reg_t usart4smen     : 1; /*!< 19 USART4SMEN: USART 4 clock enable during Sleep and Stop modes */
        mcu_reg_t usart5smen     : 1; /*!< 20 USART5SMEN: USART 5 clock enable during Sleep and Stop modes */
        mcu_reg_t i2c1smen       : 1; /*!< 21 I2C1SMEN: I2C 1 clock enable during Sleep and Stop modes */
        mcu_reg_t i2c2smen       : 1; /*!< 22 I2C2SMEN: I2C 2 clock enable during Sleep and Stop modes */
        mcu_reg_t i2c3smen       : 1; /*!< 23 I2C3SMEN: I2C 3 clock enable during Sleep and Stop modes */
        mcu_reg_t crssmen        : 1; /*!< 24 CRSSMEN: CRS clock enable during Sleep and Stop modes */
        mcu_reg_t can1smen       : 1; /*!< 25 CAN1SMEN: CAN1 clock enable during Sleep and Stop modes */
        mcu_reg_t can2smen       : 1; /*!< 26 CAN2SMEN: CAN2 clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved27     : 1; /*!< 27 Reserved, must be kept at reset value */
        mcu_reg_t pwrsmen        : 1; /*!< 28 PWRSMEN: Power interface clock enable during Sleep and Stop modes */
        mcu_reg_t dac1smen       : 1; /*!< 29 DAC1SMEN: DAC1 interface clock enable during Sleep and Stop modes */
        mcu_reg_t opampsmen      : 1; /*!< 30 OPAMPSMEN: OPAMP interface clock enable during Sleep and Stop modes */
        mcu_reg_t lptim1smen     : 1; /*!< 31 LPTIM1SMEN: Low Power Timer 1 clock enable during Sleep and Stop modes */
    }
);

/**
 * @brief APB1 peripheral clock enable in Sleep and Stop modes register 2 (RCC_APB1SMENR2)
 *
 *        Address offset: 0x7C
 *        Reset value: 0x0000 0025
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb1smen2_t,
    {
        mcu_reg_t lpuart1smen    : 1; /*!< 0 LPUART1SMEN: Low-power UART 1 clock enable during Sleep and Stop modes */
        mcu_reg_t i2c4smen       : 1; /*!< 1 I2C4SMEN: I2C 4 clock enable during Sleep and Stop modes */
        mcu_reg_t swpmi1smen     : 1; /*!< 2 SWPMI1SMEN: Single wire protocol clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved3_4    : 2; /*!< 4:3 Reserved, must be kept at reset value */
        mcu_reg_t lptim2smen     : 1; /*!< 5 LPTIM2SMEN: Low Power Timer 2 clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved6_31   : 26; /*!< 31:6 Reserved, must be kept at reset value */
    }
);

/**
 * @brief APB2 peripheral clock enable in Sleep and Stop modes register (RCC_APB2SMENR)
 *
 *        Address offset: 0x80
 *        Reset value: 0x0167 7C01
 *        Access: no wait state, word, half-word and byte access.
 */
MCU_REG_TYPE (
    rcc_apb2smen_t,
    {
        mcu_reg_t syscfgsmen     : 1; /*!< 0 SYSCFGSMEN: SYSCFG + COMP + VREFBUF clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved1_9    : 9; /*!< 9:1 Reserved, must be kept at reset value */
        mcu_reg_t sdmmc1smen     : 1; /*!< 10 SDMMC1SMEN: SDMMC clock enable during Sleep and Stop modes */
        mcu_reg_t tim1smen       : 1; /*!< 11 TIM1SMEN: TIM1 timer clock enable during Sleep and Stop modes */
        mcu_reg_t spi1smen       : 1; /*!< 12 SPI1SMEN: SPI 1 clock enable during Sleep and Stop modes */
        mcu_reg_t tim8smen       : 1; /*!< 13 TIM8SMEN: TIM8 timer clock enable during Sleep and Stop modes */
        mcu_reg_t usart1smen     : 1; /*!< 14 USART1SMEN: USART 1 clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t tim15smen      : 1; /*!< 16 TIM15SMEN: TIM15 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim16smen      : 1; /*!< 17 TIM16SMEN: TIM16 timer clock enable during Sleep and Stop modes */
        mcu_reg_t tim17smen      : 1; /*!< 18 TIM17SMEN: TIM17 timer clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved19_20  : 2; /*!< 20:19 Reserved, must be kept at reset value */
        mcu_reg_t sai1smen       : 1; /*!< 21 SAI1SMEN: Serial audio interface 1 (SAI1) clock enable during Sleep and Stop modes */
        mcu_reg_t sai2smen       : 1; /*!< 22 SAI2SMEN: Serial audio interface 2 (SAI2) clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved23     : 1; /*!< 23 Reserved, must be kept at reset value */
        mcu_reg_t dfsdm1smen     : 1; /*!< 24 DFSDM1SMEN: Digital filters for sigma-delta modulators (DFSDM1) clock enable during Sleep and Stop modes */
        mcu_reg_t Reserved25_31  : 7; /*!< 31:25 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Peripherals independent clock configuration register (RCC_CCIPR)
 *
 *        Address offset: 0x88
 *        Reset value: 0x0000 0000
 *        Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
 */
MCU_REG_TYPE (
    rcc_ccip_t,
    {
        mcu_reg_t usart1sel      : 2; /*!< 1:0 USART1SEL[1:0] USART1 clock source selection */
        mcu_reg_t usart2sel      : 2; /*!< 3:2 USART2SEL[1:0] USART2 clock source selection */
        mcu_reg_t usart3sel      : 2; /*!< 5:4 USART3SEL[1:0] USART3 clock source selection */
        mcu_reg_t usart4sel      : 2; /*!< 7:6 USART4SEL[1:0] USART4 clock source selection */
        mcu_reg_t usart5sel      : 2; /*!< 9:8 USART5SEL[1:0] USART5 clock source selection */
        mcu_reg_t lpuart1sel     : 2; /*!< 11:10 LPUART1SEL[1:0] LPUART1 clock source selection */
        mcu_reg_t i2c1sel        : 2; /*!< 13:12 I2C1SEL[1:0] I2C1 clock source selection */
        mcu_reg_t i2c2sel        : 2; /*!< 15:14 I2C2SEL[1:0] I2C2 clock source selection */
        mcu_reg_t i2c3sel        : 2; /*!< 17:16 I2C3SEL[1:0] I2C3 clock source selection */
        mcu_reg_t lptim1sel      : 2; /*!< 19:18 LPTIM1SEL[1:0] Low power timer 1 clock source selection */
        mcu_reg_t lptim2sel      : 2; /*!< 21:20 LPTIM2SEL[1:0] Low power timer 2 clock source selection */
        mcu_reg_t sai1sel        : 2; /*!< 23:22 SAI1SEL[1:0] SAI1 clock source selection */
        mcu_reg_t sai2sel        : 2; /*!< 25:24 SAI2SEL[1:0] SAI2 clock source selection */
        mcu_reg_t clk48sel       : 2; /*!< 27:26 CLK48SEL[1:0] 48 MHz clock source selection */
        mcu_reg_t adcsel         : 2; /*!< 29:28 ADCSEL[1:0] ADCs clock source selection */
        mcu_reg_t swpmi1sel      : 1; /*!< 30 SWPMI1SEL: SWPMI1 clock source selection */
        mcu_reg_t dfsdm1sel      : 1; /*!< 31 DFSDM1SEL: DFSDM1 clock source selection */
    }
);

/**
 * @brief Backup domain control register (RCC_BDCR)
 *
 *        Address offset: 0x90
 *        Reset value: 0x0000 0000, reset by Backup domain Reset, except LSCOSEL, LSCOEN
 *          and BDRST which are reset only by Backup domain power-on reset.

 *        Access: 0 ≤ wait state ≤ 3, word, half-word and byte access Wait states are inserted in case of
 *        successive accesses to this register.
 *
 *        The bits of the Backup domain control register (RCC_BDCR) are outside of the VCORE
 *        domain. As a result, after Reset, these bits are write-protected and the DBP bit in the
 *        Section 5.4.1: Power control register 1 (PWR_CR1) has to be set before these can be
 *        modified. Refer to Section 5.1.5: Battery backup domain on page 156 for further
 *        information. These bits (except LSCOSEL, LSCOEN and BDRST) are only reset after a
 *        Backup domain Reset (see Section 6.1.3: Backup domain reset). Any internal or external
 *        Reset will not have any effect on these bits.
 */
MCU_REG_TYPE (
    rcc_bdcr_t,
    {
        mcu_reg_t lseon          : 1; /*!< 0 LSEON: LSE oscillator enable */
        mcu_reg_t lserdy         : 1; /*!< 1 LSERDY: LSE oscillator ready */
        mcu_reg_t lsebyp         : 1; /*!< 2 LSEBYP: LSE oscillator bypass */
        mcu_reg_t lsedrv         : 2; /*!< 4:3 LSEDRV[1:0]: LSE oscillator's drive capability */
        mcu_reg_t lsecsson       : 1; /*!< 5 LSECSSON: CSS on LSE enable */
        mcu_reg_t lsecssd        : 1; /*!< 6 LSECSSD: CSS on LSE failure Detection */
        mcu_reg_t Reserved7      : 1; /*!< 7 Reserved, must be kept at reset value */
        mcu_reg_t rtcsel         : 2; /*!< 9:8 rtcsel[1:0] RTC clock source selection */
        mcu_reg_t Reserved10_14  : 5; /*!< 14:10 Reserved, must be kept at reset value */
        mcu_reg_t rtcen          : 1; /*!< 15 RTCEN: RTC clock enable */
        mcu_reg_t bdrst          : 1; /*!< 16 BDRST: Backup domain software reset */
        mcu_reg_t Reserved17_23  : 7; /*!< 23:17 Reserved, must be kept at reset value */
        mcu_reg_t lscoen         : 1; /*!< 24 LSCOEN: Low speed clock output enable */
        mcu_reg_t lscosel        : 1; /*!< 25 LSCOSEL: Low speed clock output selection */
        mcu_reg_t Reserved26_31  : 6; /*!< 31:26 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Control/status register (RCC_CSR)
 *
 *        Address offset: 0x94
 *        Reset value: 0x0C00 0600, reset by system reset, except reset flags by power reset only.
 *        Access: 0 ≤ wait state ≤ 3, word, half-word and byte access
 *        Wait states are inserted in case of successive accesses to this register.
 */
MCU_REG_TYPE (
    rcc_csr_t,
    {
        mcu_reg_t lsion          : 1; /*!< 0 LSION: LSI oscillator enable */
        mcu_reg_t lsirdy         : 1; /*!< 1 LSIRDY: LSI oscillator ready */
        mcu_reg_t Reserved2_7    : 6; /*!< 7:2 Reserved, must be kept at reset value */
        mcu_reg_t msisrange      : 4; /*!< 8:11 MSI[3:0] MSI range after Standby mode */
        mcu_reg_t Reserved12_22  : 11; /*!< 22:12 Reserved, must be kept at reset value */
        mcu_reg_t rmv            : 1; /*!< 23 RMVF: Remove reset flag */
        mcu_reg_t fwrst          : 1; /*!< 24 FWRSTF: Firewall reset flag */
        mcu_reg_t oblrst         : 1; /*!< 25 OBLRSTF: Option byte loader reset flag */
        mcu_reg_t pinrst         : 1; /*!< 26 PINRSTF: PIN reset flag */
        mcu_reg_t borrst         : 1; /*!< 27 BORRSTF: BOR reset flag */
        mcu_reg_t sftrst         : 1; /*!< 28 SFTRSTF: Software reset flag */
        mcu_reg_t iwdgrst        : 1; /*!< 29 IWDGRSTF: Independent watchdog reset flag */
        mcu_reg_t wwdgrst        : 1; /*!< 30 WWDGRSTF: Window watchdog reset flag */
        mcu_reg_t lpwrrst        : 1; /*!< 31 LPWRRSTF: Low-power reset flag */
    }
);

/**
 * @brief RCC register map
 */
typedef __packed struct
{
    volatile rcc_cr_t           CR;             /*!< 01 0x00 */
    volatile rcc_icsc_t         ICSC;           /*!< 02 0x04 */
    volatile rcc_cfg_t          CFG;            /*!< 03 0x08 */
    volatile rcc_pllcfg_t       PLLCFG;         /*!< 04 0x0C */
    volatile rcc_pllsai1cfg_t   PLLSAI1CFG;     /*!< 05 0x10 */
    volatile rcc_pllsai2cfg_t   PLLSAI2CFG;     /*!< 06 0x14 */
    volatile rcc_cie_t          CIE;            /*!< 07 0x18 */
    volatile rcc_cif_t          CIF;            /*!< 08 0x1C */
    volatile rcc_cic_t          CIC;            /*!< 09 0x20 */
    volatile mcu_reg_t          RESERVED0;      /*!< 10 0x24 */
    volatile rcc_ahb1rst_t      AHB1RST;        /*!< 11 0x28 */
    volatile rcc_ahb2rst_t      AHB2RST;        /*!< 12 0x2C */
    volatile rcc_ahb3rst_t      AHB3RST;        /*!< 13 0x30 */
    volatile mcu_reg_t          RESERVED1;      /*!< 14 0x34 */
    volatile rcc_apb1rst1_t     APB1RST1;       /*!< 15 0x38 */
    volatile rcc_apb1rst2_t     APB1RST2;       /*!< 16 0x3C */
    volatile rcc_apb2rst_t      APB2RST;        /*!< 17 0x40 */
    volatile mcu_reg_t          RESERVED2;      /*!< 18 0x44 */
    volatile rcc_ahb1en_t       AHB1EN;         /*!< 19 0x48 */
    volatile rcc_ahb2en_t       AHB2EN;         /*!< 20 0x4C */
    volatile rcc_ahb3en_t       AHB3EN;         /*!< 21 0x50 */
    volatile mcu_reg_t          RESERVED3;      /*!< 22 0x54 */
    volatile rcc_apb1en1_t      APB1EN1;        /*!< 23 0x58 */
    volatile rcc_apb1en2_t      APB1EN2;        /*!< 24 0x5C */
    volatile rcc_apb2en_t       APB2EN;         /*!< 25 0x60 */
    volatile mcu_reg_t          RESERVED4;      /*!< 26 0x64 */
    volatile rcc_ahb1smen_t     AHB1SMEN;       /*!< 27 0x68 */
    volatile rcc_ahb2smen_t     AHB2SMEN;       /*!< 28 0x6C */
    volatile rcc_ahb3smen_t     AHB3SMEN;       /*!< 29 0x70 */
    volatile mcu_reg_t          RESERVED5;      /*!< 30 0x74 */
    volatile rcc_apb1smen1_t    APB1SMEN1;      /*!< 31 0x78 */
    volatile rcc_apb1smen2_t    APB1SMEN2;      /*!< 32 0x7C */
    volatile rcc_apb2smen_t     APB2SMEN;       /*!< 33 0x80 */
    volatile mcu_reg_t          RESERVED6;      /*!< 34 0x84 */
    volatile rcc_ccip_t         CCIP;           /*!< 35 0x88 */
    volatile mcu_reg_t          RESERVED7;      /*!< 36 0x8C */
    volatile rcc_bdcr_t         BDCR;           /*!< 37 0x90 */
    volatile rcc_csr_t          CSR;            /*!< 38 0x94 */
} rcc_regs_t;

CHECK_REG_MAP(rcc_regs_t, 38);

#define RCC_REGS_ADDR (rcc_regs_t*)RCC_BASE

#endif  // STM32L475_RCC_REG_H
