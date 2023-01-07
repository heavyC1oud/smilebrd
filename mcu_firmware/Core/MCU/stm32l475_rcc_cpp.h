/**
 * @file stm32l475_rcc_cpp.h
 *
 *       Reset and clock control
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_RCC_CPP_H
#define STM32L475_RCC_CPP_H

#include "stm32l4xx.h"
#include "stm32l475_rcc_reg.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)12000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

typedef enum
{
    RCC_CLOCK_SYS = 0,
    RCC_CLOCK_AHB = 1,
    RCC_CLOCK_APB1 = 2,
    RCC_CLOCK_APB2 = 3,
} rcc_clocks_t;

typedef enum
{
    RCC_SYSCLOCK_MSI   = 0, //!< MSI selected as system clock
    RCC_SYSCLOCK_HSI16 = 1, //!< HSI16 oscillator selected as system clock
    RCC_SYSCLOCK_HSE   = 2, //!< HSE oscillator selected as system clock
    RCC_SYSCLOCK_PLL   = 3, //!< PLL selected as system clock
} rcc_sysclock_src_t;

typedef enum
{
    RCC_AHBCLOCK_DIV1   = 0,  //!< 0xxx: system clock not divided
    RCC_AHBCLOCK_DIV2   = 8,  //!< 1000: system clock divided by 2
    RCC_AHBCLOCK_DIV4   = 9,  //!< 1001: system clock divided by 4
    RCC_AHBCLOCK_DIV8   = 10, //!< 1010: system clock divided by 8
    RCC_AHBCLOCK_DIV16  = 11, //!< 1011: system clock divided by 16
    RCC_AHBCLOCK_DIV64  = 12, //!< 1100: system clock divided by 64
    RCC_AHBCLOCK_DIV128 = 13, //!< 1101: system clock divided by 128
    RCC_AHBCLOCK_DIV258 = 14, //!< 1110: system clock divided by 256
    RCC_AHBCLOCK_DIV512 = 15, //!< 1111: system clock divided by 512
} rcc_ahbclock_div_t;

typedef enum
{
    RCC_APBCLOCK_DIV1  = 0, //!< 0xx: AHB clock not divided
    RCC_APBCLOCK_DIV2  = 4, //!< 100: AHB clock divided by 2
    RCC_APBCLOCK_DIV4  = 5, //!< 101: AHB clock divided by 4
    RCC_APBCLOCK_DIV8  = 6, //!< 110: AHB clock divided by 8
    RCC_APBCLOCK_DIV16 = 7, //!< 111: AHB clock divided by 16
} rcc_apbclock_div_t;

typedef enum
{
    RCC_MSI_RANGE_100_KHZ = 0,  //!< 0000: range 0 around 100 kHz
    RCC_MSI_RANGE_200_KHZ = 1,  //!< 0001: range 1 around 200 kHz
    RCC_MSI_RANGE_400_KHZ = 2,  //!< 0010: range 2 around 400 kHz
    RCC_MSI_RANGE_800_KHZ = 3,  //!< 0011: range 3 around 800 kHz
    RCC_MSI_RANGE_1_MHZ   = 4,  //!< 0100: range 4 around 1 MHz
    RCC_MSI_RANGE_2_MHZ   = 5,  //!< 0101: range 5 around 2 MHz
    RCC_MSI_RANGE_4_MHZ   = 6,  //!< 0110: range 6 around 4 MHz
    RCC_MSI_RANGE_8_MHZ   = 7,  //!< 0111: range 7 around 8 MHz
    RCC_MSI_RANGE_16_MHZ  = 8,  //!< 1000: range 8 around 16 MHz
    RCC_MSI_RANGE_24_MHZ  = 9,  //!< 1001: range 9 around 24 MHz
    RCC_MSI_RANGE_32_MHZ  = 10,  //!< 1010: range 10 around 32 MHz
    RCC_MSI_RANGE_48_MHZ  = 11,  //!< 1011: range 11 around 48 MHz
} rcc_msi_range_t;

static const uint32_t msi_range_table[] = {
    0x000186A0, 0x00030D40, 0x00061A80, 0x000C3500,
    0x000F4240, 0x001E8480, 0x003D0900, 0x007A1200,
    0x00F42400, 0x016E3600, 0x01E84800, 0x02DC6C00
    };

typedef enum
{
    RCC_PLL_NO_CLOCK = 0, //!< No clock sent to PLL
    RCC_PLL_MSI      = 1, //!< MSI clock selected as PLL clock entry
    RCC_PLL_HSI16    = 2, //!< HSI16 clock selected as PLL clock entry
    RCC_PLL_HSE      = 3, //!< HSE clock selected as PLL clock entry
} rcc_pll_src_t;

typedef enum
{
    RCC_PLL_M_1 = 0, //division by 1
    RCC_PLL_M_2 = 1, //division by 2
    RCC_PLL_M_3 = 2, //division by 3
    RCC_PLL_M_4 = 3, //division by 4
    RCC_PLL_M_5 = 4, //division by 5
    RCC_PLL_M_6 = 5, //division by 6
    RCC_PLL_M_7 = 6, //division by 7
    RCC_PLL_M_8 = 7, //division by 8
} rcc_pll_m_t;

typedef enum
{
    RCC_PLL_P_7  = 0, //division by 7
    RCC_PLL_P_17 = 1, //division by 17
} rcc_pll_p_t;

typedef enum
{
    RCC_PLL_Q_2 = 0, //division by 2
    RCC_PLL_Q_4 = 1, //division by 4
    RCC_PLL_Q_6 = 2, //division by 6
    RCC_PLL_Q_8 = 3, //division by 8
} rcc_pll_q_t;

typedef enum
{
    RCC_PLL_R_2 = 0, //division by 2
    RCC_PLL_R_4 = 1, //division by 4
    RCC_PLL_R_6 = 2, //division by 6
    RCC_PLL_R_8 = 3, //division by 8
} rcc_pll_r_t;

typedef enum
{
    RCC_RTC_NO_CLOCK = 0,        //No clock
    RCC_RTC_LSE      = 1,        //LSE oscillator clock used as the RTC clock
    RCC_RTC_LSI      = 2,        //LSI oscillator clock used as the RTC clock
    RCC_RTC_HSE      = 3,        //HSE oscillator clock divided by a programmable prescaler
} rcc_rtcclock_src_t;

typedef enum
{
    RCC_MCO_DIS    = 0,    //!< MCO output disabled
    RCC_MCO_SYSCLK = 1,    //!< SYSCLK selected
    RCC_MCO_MSI    = 2,     //!< MSI selected
    RCC_MCO_HSI16  = 3,    //!< HSI16 selected
    RCC_MCO_HSE    = 4,    //!< HSE selected
    RCC_MCO_PLL    = 5,    //!< PLL selected
    RCC_MCO_LSI    = 6,    //!< LSI selected
    RCC_MCO_LSE    = 7,    //!< LSE selected
} rcc_mco_src_t;

typedef enum
{
    RCC_MCO_DIV_0  = 0, //!< no division
    RCC_MCO_DIV_2  = 1, //!< division by 2
    RCC_MCO_DIV_4  = 2, //!< division by 4
    RCC_MCO_DIV_8  = 3, //!< division by 8
    RCC_MCO_DIV_16 = 4, //!< division by 16
} rcc_mco_div_t;

typedef enum
{
    RCC_APB1_PERIPH_TIM2    = 0x00000001,
    RCC_APB1_PERIPH_TIM3    = 0x00000002,
    RCC_APB1_PERIPH_TIM4    = 0x00000004,
    RCC_APB1_PERIPH_TIM5    = 0x00000008,
    RCC_APB1_PERIPH_TIM6    = 0x00000010,
    RCC_APB1_PERIPH_TIM7    = 0x00000020,
    RCC_APB1_PERIPH_LCD     = 0x00000200,
    RCC_APB1_PERIPH_RTCAPB  = 0x00000400,
    RCC_APB1_PERIPH_WWDG    = 0x00000800,
    RCC_APB1_PERIPH_SPI2    = 0x00004000,
    RCC_APB1_PERIPH_SPI3    = 0x00008000,
    RCC_APB1_PERIPH_UART2   = 0x00020000,
    RCC_APB1_PERIPH_UART3   = 0x00040000,
    RCC_APB1_PERIPH_UART4   = 0x00080000,
    RCC_APB1_PERIPH_UART5   = 0x00100000,
    RCC_APB1_PERIPH_I2C1    = 0x00200000,
    RCC_APB1_PERIPH_I2C2    = 0x00400000,
    RCC_APB1_PERIPH_I2C3    = 0x00800000,
    RCC_APB1_PERIPH_CRS     = 0x01000000,
    RCC_APB1_PERIPH_CAN1    = 0x02000000,
    RCC_APB1_PERIPH_CAN2    = 0x04000000,
    RCC_APB1_PERIPH_PWR     = 0x10000000,
    RCC_APB1_PERIPH_DAC1    = 0x20000000,
    RCC_APB1_PERIPH_OPAMP   = 0x40000000,
    RCC_APB1_PERIPH_LPTIM1  = 0x80000000,
} rcc_apb1_1_periph_t;

typedef enum
{
    RCC_APB1_PERIPH_LPUART1 = 0x00000001,
    RCC_APB1_PERIPH_I2C4    = 0x00000002,
    RCC_APB1_PERIPH_SWPMI1  = 0x00000004,
    RCC_APB1_PERIPH_LPTIM2  = 0x80000020,
} rcc_apb1_2_periph_t;

typedef enum
{
    RCC_APB2_PERIPH_SYSCFG  = 0x00000001,
    RCC_APB2_PERIPH_FW      = 0x00000080,
    RCC_APB2_PERIPH_SDMMC1  = 0x00000400,
    RCC_APB2_PERIPH_TIM1    = 0x00000800,
    RCC_APB2_PERIPH_SPI1    = 0x00001000,
    RCC_APB2_PERIPH_TIM8    = 0x00002000,
    RCC_APB2_PERIPH_UART1   = 0x00004000,
    RCC_APB2_PERIPH_TIM15   = 0x00010000,
    RCC_APB2_PERIPH_TIM16   = 0x00020000,
    RCC_APB2_PERIPH_TIM17   = 0x00040000,
    RCC_APB2_PERIPH_SAI1    = 0x00200000,
    RCC_APB2_PERIPH_SAI2    = 0x00400000,
    RCC_APB2_PERIPH_DFSDM1  = 0x01000000,
} rcc_apb2_periph_t;

typedef enum
{
    RCC_AHB1_PERIPH_DMA1    = 0x00000001,
    RCC_AHB1_PERIPH_DMA2    = 0x00000002,
    RCC_AHB1_PERIPH_FLASH   = 0x00000100,
    RCC_AHB1_PERIPH_CRC     = 0x00001000,
    RCC_AHB1_PERIPH_TSC     = 0x00010000,
    RCC_AHB1_PERIPH_DMA2D   = 0x00020000,
} rcc_ahb1_periph_t;

typedef enum
{
    RCC_AHB2_PERIPH_GPIOA   = 0x00000001,
    RCC_AHB2_PERIPH_GPIOB   = 0x00000002,
    RCC_AHB2_PERIPH_GPIOC   = 0x00000004,
    RCC_AHB2_PERIPH_GPIOD   = 0x00000008,
    RCC_AHB2_PERIPH_GPIOE   = 0x00000010,
    RCC_AHB2_PERIPH_GPIOF   = 0x00000020,
    RCC_AHB2_PERIPH_GPIOG   = 0x00000040,
    RCC_AHB2_PERIPH_GPIOH   = 0x00000080,
    RCC_AHB2_PERIPH_GPIOI   = 0x00000100,
    RCC_AHB2_PERIPH_OTGFS   = 0x00001000,
    RCC_AHB2_PERIPH_ADC     = 0x00002000,
    RCC_AHB2_PERIPH_DCMI    = 0x00004000,
} rcc_ahb2_periph_t;

typedef enum
{
    RCC_AHB3_PERIPH_FMC    = 0x00000001,
    RCC_AHB3_PERIPH_QSPI   = 0x00000100,
} rcc_ahb3_periph_t;

class rcc_t
{
    public:     // REMOVE
    static struct rcc_base_reg
    {
        rcc_regs_t* operator-> ()
        {
            return RCC_REGS_ADDR;
        }
    } RCC_REGS;

public:
    static void deinit()
    {
        rcc_t::hsi_on();
        // Reset CFGR register
        RCC_REGS->CFG.d32 = 0x00000000;
        rcc_t::pll_off();
        rcc_t::hse_off();
        rcc_t::msi_off();
        // Reset PLLCFGR register
        RCC_REGS->PLLCFG.d32 = 0x00001000;
        rcc_t::hse_bypass_off();
        // Disable all interrupts
        RCC_REGS->CIE.d32 = 0x00000000;
    };

    inline static void hse_on() {RCC_REGS->CR.hseon = 1;}
    inline static void hse_off() {RCC_REGS->CR.hseon = 0;}
    inline static void hse_bypass_on() {RCC_REGS->CR.hsebyp = 1;}
    inline static void hse_bypass_off() {RCC_REGS->CR.hsebyp = 0;}
    inline static bool is_hse_ready() {return (RCC_REGS->CR.hserdy == 1);}

    inline static void hsi_on() {RCC_REGS->CR.hsion = 1;}
    inline static void hsi_off() {RCC_REGS->CR.hsion = 0;}
    inline static void hsi_calibrate(uint8_t data) {RCC_REGS->ICSC.hsical = data;}
    inline static bool is_hsi_ready() {return (RCC_REGS->CR.hsirdy == 1);}

    inline static void msi_on() {RCC_REGS->CR.msion = 1;}
    inline static void msi_off() {RCC_REGS->CR.msion = 0;}
    inline static void msi_calibrate(uint8_t data) {RCC_REGS->ICSC.msical = data;}
    inline static void msi_set_range(rcc_msi_range_t data) {RCC_REGS->CR.msirange = data;}

    inline static void lse_on() {RCC_REGS->BDCR.lseon = 1;}
    inline static void lse_off() {RCC_REGS->BDCR.lseon = 0;}
    inline static void lse_bypass_on() {RCC_REGS->BDCR.lsebyp = 1;}
    inline static void lse_bypass_off() {RCC_REGS->BDCR.lsebyp = 0;}
    inline static bool is_lse_ready() {return (RCC_REGS->BDCR.lserdy == 1);}

    inline static void lsi_on() {RCC_REGS->CSR.lsion = 1;}
    inline static void lsi_off() {RCC_REGS->CSR.lsion = 0;}
    inline static bool is_lsi_ready() {return (RCC_REGS->CSR.lsirdy == 1);}

    inline static void pll_on() {RCC_REGS->CR.pllon = 1;}
    inline static void pll_off() {RCC_REGS->CR.pllon = 0;}

    inline static void pll_config(rcc_pll_src_t source, rcc_pll_m_t m, uint32_t n, rcc_pll_p_t p, rcc_pll_q_t q, rcc_pll_r_t r)
    {
        RCC_REGS->PLLCFG.d32 = source | (m << 4) | (n << 8) | (p << 17) | (q << 21) | (r << 25);
    };

    inline static void pll_source(rcc_pll_src_t source) {RCC_REGS->PLLCFG.pllsrc = source;}

    inline static rcc_pll_src_t get_pll_source() {return (rcc_pll_src_t)RCC_REGS->PLLCFG.pllsrc;}

    inline static void pll_m(rcc_pll_m_t m) {RCC_REGS->PLLCFG.pllm = m;}
    inline static void pll_n(uint8_t n) {RCC_REGS->PLLCFG.plln = n;}

    inline static void pll_p(rcc_pll_p_t p) {RCC_REGS->PLLCFG.pllp = p;}
    inline static void pll_p_on() {RCC_REGS->PLLCFG.pllpen = 1;}
    inline static void pll_p_off() {RCC_REGS->PLLCFG.pllpen = 0;}

    inline static void pll_q(rcc_pll_q_t q) {RCC_REGS->PLLCFG.pllq = q;}
    inline static void pll_q_on() {RCC_REGS->PLLCFG.pllqen = 1;}
    inline static void pll_q_off() {RCC_REGS->PLLCFG.pllqen = 0;}

    inline static void pll_r(rcc_pll_r_t r) {RCC_REGS->PLLCFG.pllr = r;}
    inline static void pll_r_on() {RCC_REGS->PLLCFG.pllren = 1;}
    inline static void pll_r_off() {RCC_REGS->PLLCFG.pllren = 0;}

    inline static bool is_pll_ready() {return (RCC_REGS->CR.pllrdy == 1);}

    inline static void css_on() {RCC_REGS->CR.csson = 1;}

    inline static void mco_source(rcc_mco_src_t source) {RCC_REGS->CFG.mcosel = source;}
    inline static void mco_div(rcc_mco_div_t div) {RCC_REGS->CFG.mcopre = div;}

    inline static void sysclock_source(rcc_sysclock_src_t source) {RCC_REGS->CFG.sw = source;}

    inline static rcc_sysclock_src_t get_sysclock_source() {return (rcc_sysclock_src_t)RCC_REGS->CFG.sws;}

    inline static void ahbclock_div(rcc_ahbclock_div_t div) {RCC_REGS->CFG.hpre = div;}
    inline static void apb1clock_div(rcc_apbclock_div_t div) {RCC_REGS->CFG.ppre1 = div;}
    inline static void apb2clock_div(rcc_apbclock_div_t div) {RCC_REGS->CFG.ppre2 = div;}

    inline static uint32_t get_hse_freq() {return HSE_VALUE;}
    inline static uint32_t get_hsi_freq() {return 16000000;}
    inline static uint32_t get_lsi_freq() {return 32000;}
    inline static uint32_t get_msi_freq() {return msi_range_table[RCC_REGS->CR.msirange];}


    /**
     * @brief Получить частоту тактирования для указанной системной шины
     *
     * @param clock : rcc_clocks_t RCC_CLOCK_SYS  : core bus
     *                             RCC_CLOCK_AHB  : AHB bus
     *                             RCC_CLOCK_APB1 : APB1 bus
     *                             RCC_CLOCK_APB2 : APB2 bus
     *
     * @return uint32_t частота тактироания в Гц.
     */
    static uint32_t get_clock(rcc_clocks_t clock)
    {
        const uint8_t APBAHBPrescTable[16] =
            {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
        uint32_t sys = rcc_t::get_hsi_freq();
        uint32_t ahb = 0;
        /* Расчитаем частоту тактирования системной шины (RCC_CLOCK_SYS)
           в зависимости от источника тактирования */
        switch (rcc_t::get_sysclock_source())
        {
        case RCC_SYSCLOCK_HSE:
            // тактируемся от HSE, внешний источник тактирования
            sys = rcc_t::get_hse_freq();
            break;
        case RCC_SYSCLOCK_PLL:
        {   // тактируемся через PLL
            uint32_t freq   = 0;
            uint32_t vco    = 0;
            uint32_t r      = 0;
            // получить источник тактирования для PLL
            switch(RCC_REGS->PLLCFG.pllsrc) {
            case RCC_PLL_NO_CLOCK:
                freq = 0;
                break;
            case RCC_PLL_MSI:
                freq = rcc_t::get_msi_freq();
                break;
            case RCC_PLL_HSI16:
                freq = rcc_t::get_hsi_freq();
                break;
            case RCC_PLL_HSE:
                freq = rcc_t::get_hse_freq();
                break;
            default:
                freq = rcc_t::get_hsi_freq();
                break;
            }

            vco = (freq / (RCC_REGS->PLLCFG.pllm + 1));
            vco *= RCC_REGS->PLLCFG.plln;
            r = (RCC_REGS->PLLCFG.pllr + 1) * 2;
            sys = vco/r;
            break;
        }
        case RCC_SYSCLOCK_MSI:
            sys = rcc_t::get_msi_freq();
            break;
        case RCC_SYSCLOCK_HSI16:
        default:
            // тактируемся от HSI, внутренний RC генератор
            sys = rcc_t::get_hsi_freq();
            break;
        }

        if(clock == RCC_CLOCK_SYS) {
            return sys;
        }

        ahb = sys >> APBAHBPrescTable[RCC_REGS->CFG.hpre];
        if(clock == RCC_CLOCK_AHB) {
            return ahb;
        }

        if(clock == RCC_CLOCK_APB1) {
            return (ahb >> APBAHBPrescTable[RCC_REGS->CFG.ppre1]);
        }

        return (ahb >> APBAHBPrescTable[RCC_REGS->CFG.ppre2]);
    }

    inline static void rtc_on() {RCC_REGS->BDCR.rtcen = 1;}
    inline static void rtc_off() {RCC_REGS->BDCR.rtcen = 0;}

    inline static void rtcclock_source(rcc_rtcclock_src_t source) {RCC_REGS->BDCR.rtcsel = source;}

    inline static void bkup_reset_on() {RCC_REGS->BDCR.bdrst = 1;}
    inline static void bkup_reset_off() {RCC_REGS->BDCR.bdrst = 0;}

    inline static void ahb1_periph_on(rcc_ahb1_periph_t data) {RCC_REGS->AHB1EN.d32 |= data;}
    inline static void ahb1_periph_off(rcc_ahb1_periph_t data) {RCC_REGS->AHB1EN.d32 &= ~data;}

    inline static void ahb2_periph_on(rcc_ahb2_periph_t data) {RCC_REGS->AHB2EN.d32 |= data;}
    inline static void ahb2_periph_off(rcc_ahb2_periph_t data) {RCC_REGS->AHB2EN.d32 &= ~data;}

    inline static void ahb3_periph_on(rcc_ahb3_periph_t data) {RCC_REGS->AHB3EN.d32 |= data;}
    inline static void ahb3_periph_off(rcc_ahb3_periph_t data) {RCC_REGS->AHB3EN.d32 &= ~data;}

    inline static void apb1en1_periph_on(rcc_apb1_1_periph_t data) {RCC_REGS->APB1EN1.d32 |= data;}
    inline static void apb1en1_periph_off(rcc_apb1_1_periph_t data) {RCC_REGS->APB1EN1.d32 &= ~data;}

    inline static void apb1en2_periph_on(rcc_apb1_2_periph_t data) {RCC_REGS->APB1EN2.d32 |= data;}
    inline static void apb1en2_periph_off(rcc_apb1_2_periph_t data) {RCC_REGS->APB1EN2.d32 &= ~data;}

    inline static void apb2_periph_on(rcc_apb2_periph_t data) {RCC_REGS->APB2EN.d32 |= data;}
    inline static void apb2_periph_off(rcc_apb2_periph_t data) {RCC_REGS->APB2EN.d32 &= ~data;}

    inline static bool is_ahb1_periph_on(rcc_ahb1_periph_t data) {return ((RCC_REGS->AHB1EN.d32 & data) != 0);}
    inline static bool is_ahb2_periph_on(rcc_ahb2_periph_t data) {return ((RCC_REGS->AHB2EN.d32 & data) != 0);}
    inline static bool is_ahb3_periph_on(rcc_ahb3_periph_t data) {return ((RCC_REGS->AHB3EN.d32 & data) != 0);}
    inline static bool is_apb1en1_periph_on(rcc_apb1_1_periph_t data) {return ((RCC_REGS->APB1EN1.d32 & data) != 0);}
    inline static bool is_apb1en2_periph_on(rcc_apb1_2_periph_t data) {return ((RCC_REGS->APB1EN2.d32 & data) != 0);}
    inline static bool is_apb2_periph_on(rcc_apb2_periph_t data) {return ((RCC_REGS->APB2EN.d32 & data) != 0);}

    inline static void ahb1_periph_reset_on(rcc_ahb1_periph_t data) {RCC_REGS->AHB1RST.d32 |= data;}
    inline static void ahb1_periph_reset_off(rcc_ahb1_periph_t data) {RCC_REGS->AHB1RST.d32 &= ~data;}

    inline static void ahb2_periph_reset_on(rcc_ahb2_periph_t data) {RCC_REGS->AHB2RST.d32 |= data;}
    inline static void ahb2_periph_reset_off(rcc_ahb2_periph_t data) {RCC_REGS->AHB2RST.d32 &= ~data;}

    inline static void ahb3_periph_reset_on(rcc_ahb3_periph_t data) {RCC_REGS->AHB3RST.d32 |= data;}
    inline static void ahb3_periph_reset_off(rcc_ahb3_periph_t data) {RCC_REGS->AHB3RST.d32 &= ~data;}

    inline static void apb1rst1_periph_reset_on(rcc_apb1_1_periph_t data) {RCC_REGS->APB1RST1.d32 |= data;}
    inline static void apb1rst1_periph_reset_off(rcc_apb1_1_periph_t data) {RCC_REGS->APB1RST1.d32 &= ~data;}

    inline static void apb1rst2_periph_reset_on(rcc_apb1_2_periph_t data) {RCC_REGS->APB1RST2.d32 |= data;}
    inline static void apb1rst2_periph_reset_off(rcc_apb1_2_periph_t data) {RCC_REGS->APB1RST2.d32 &= ~data;}

    inline static void apb2_periph_reset_on(rcc_apb2_periph_t data) {RCC_REGS->APB2RST.d32 |= data;}
    inline static void apb2_periph_reset_off(rcc_apb2_periph_t data) {RCC_REGS->APB2RST.d32 &= ~data;}

    inline static bool is_fw_reset() {return (RCC_REGS->CSR.fwrst == 1);}
    inline static bool is_obl_reset() {return (RCC_REGS->CSR.oblrst == 1);}
    inline static bool is_pin_reset() {return (RCC_REGS->CSR.pinrst == 1);}
    inline static bool is_bor_reset() {return (RCC_REGS->CSR.borrst == 1);}
    inline static bool is_soft_reset() {return (RCC_REGS->CSR.sftrst == 1);}
    inline static bool is_iwdg_reset() {return (RCC_REGS->CSR.iwdgrst == 1);}
    inline static bool is_wwdg_reset() {return (RCC_REGS->CSR.wwdgrst == 1);}
    inline static bool is_low_power_reset() {return (RCC_REGS->CSR.lpwrrst == 1);}
    inline static void clear_reset_flags() {RCC_REGS->CSR.rmv = 1;}

    inline static bool is_lsirdy_irq() {return (RCC_REGS->CIF.lsirdy == 1);}
    inline static bool is_lserdy_irq() {return (RCC_REGS->CIF.lserdy == 1);}
    inline static bool is_msirdy_irq() {return (RCC_REGS->CIF.msirdy == 1);}
    inline static bool is_hsirdy_irq() {return (RCC_REGS->CIF.hsirdy == 1);}
    inline static bool is_hserdy_irq() {return (RCC_REGS->CIF.hserdy == 1);}
    inline static bool is_pllrdy_irq() {return (RCC_REGS->CIF.pllrdy == 1);}
    inline static bool is_pllsai1rdy_irq() {return (RCC_REGS->CIF.pllsai1rdy == 1);}
    inline static bool is_pllsai2rdy_irq() {return (RCC_REGS->CIF.pllsai2rdy == 1);}
    inline static bool is_css_irq() {return (RCC_REGS->CIF.css == 1);}
    inline static bool is_lsecss_irq() {return (RCC_REGS->CIF.lsecss == 1);}
    inline static bool is_hsi48rdy_irq() {return (RCC_REGS->CIF.hsi48rdy == 1);}

    inline static void lsirdy_irq_en() {RCC_REGS->CIE.lsirdyie = 1;}
    inline static void lserdy_irq_en() {RCC_REGS->CIE.lserdyie = 1;}
    inline static void msirdy_irq_en() {RCC_REGS->CIE.msirdyie = 1;}
    inline static void hsirdy_irq_en() {RCC_REGS->CIE.hsirdyie = 1;}
    inline static void hserdy_irq_en() {RCC_REGS->CIE.hserdyie = 1;}
    inline static void pllrdy_irq_en() {RCC_REGS->CIE.pllrdyie = 1;}
    inline static void pllsai1rdy_irq_en() {RCC_REGS->CIE.pllsai1rdyie = 1;}
    inline static void pllsai2rdy_irq_en() {RCC_REGS->CIE.pllsai2rdyie = 1;}
    inline static void lsecss_irq_en() {RCC_REGS->CIE.lsecssie = 1;}
    inline static void hsi48rdy_irq_en() {RCC_REGS->CIE.hsi48rdyie = 1;}

    inline static void lsirdy_irq_dis() {RCC_REGS->CIE.lsirdyie = 0;}
    inline static void lserdy_irq_dis() {RCC_REGS->CIE.lserdyie = 0;}
    inline static void msirdy_irq_dis() {RCC_REGS->CIE.msirdyie = 0;}
    inline static void hsirdy_irq_dis() {RCC_REGS->CIE.hsirdyie = 0;}
    inline static void hserdy_irq_dis() {RCC_REGS->CIE.hserdyie = 0;}
    inline static void pllrdy_irq_dis() {RCC_REGS->CIE.pllrdyie = 0;}
    inline static void pllsai0rdy_irq_dis() {RCC_REGS->CIE.pllsai1rdyie = 0;}
    inline static void pllsai2rdy_irq_dis() {RCC_REGS->CIE.pllsai2rdyie = 0;}
    inline static void lsecss_irq_dis() {RCC_REGS->CIE.lsecssie = 0;}
    inline static void hsi48rdy_irq_dis() {RCC_REGS->CIE.hsi48rdyie = 0;}

    inline static void lsirdy_irq_clear() {RCC_REGS->CIC.lsirdyc = 1;}
    inline static void lserdy_irq_clear() {RCC_REGS->CIC.lserdyc = 1;}
    inline static void msirdy_irq_clear() {RCC_REGS->CIC.msirdyc = 1;}
    inline static void hsirdy_irq_clear() {RCC_REGS->CIC.hsirdyc = 1;}
    inline static void hserdy_irq_clear() {RCC_REGS->CIC.hserdyc = 1;}
    inline static void pllrdy_irq_clear() {RCC_REGS->CIC.pllrdyc = 1;}
    inline static void pllsai1rdy_irq_clear() {RCC_REGS->CIC.pllsai1rdyc = 1;}
    inline static void pllsai2rdy_irq_clear() {RCC_REGS->CIC.pllsai2rdyc = 1;}
    inline static void css_irq_clear() {RCC_REGS->CIC.cssc = 1;}
    inline static void lsecss_irq_clear() {RCC_REGS->CIC.lsecssc = 1;}
    inline static void hsi48rdy_irq_clear() {RCC_REGS->CIC.hsi48rdyc = 1;}
};

#endif  // STM32L475_RCC_CPP_H
