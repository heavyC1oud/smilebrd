/**
 * @file stm32l475_tmr_reg.h
 *
 *       Timer registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_TMR_REG_H
#define STM32L475_TMR_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief TIMx control register 1 (TIMx_CR1)
 *        Address offset: 0x00
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_cr1_t,
    {
        mcu_reg_t cen           : 1;  //!< 0 CEN: Counter enable
        mcu_reg_t udis          : 1;  //!< 1 UDIS: Update disable
        mcu_reg_t urs           : 1;  //!< 2 URS: Update request source
        mcu_reg_t opm           : 1;  //!< 3 OPM: One-pulse mode
        mcu_reg_t dir           : 1;  //!< 4 DIR: Direction
        mcu_reg_t Reserved5_6   : 2;  //!< 5:4 Reserved, must be kept at reset value
        mcu_reg_t arpe          : 1;  //!< 7 ARPE: Auto-reload preload enable
        mcu_reg_t ckd           : 2;  //!< 9:8 CKD[1:0] Clock division.
        mcu_reg_t Reserved10    : 1;  //!< 10 Reserved, must be kept at reset value
        mcu_reg_t uifremap      : 1;  //!< 11 UIFREMAP: UIF status bit remapping
        mcu_reg_t Reserved12_31 : 20; //!< 31:12 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx control register 2 (TIMx_CR2)
 *        Address offset: 0x04
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_cr2_t,
    {
        mcu_reg_t ccpc          : 1;  //!< 0 CCPC: Capture/compare preloaded control
        mcu_reg_t Reserved1     : 1;  //!< 1 Reserved, must be kept at reset value
        mcu_reg_t ccus          : 1;  //!< 2 CCUS: Capture/compare control update selection
        mcu_reg_t ccds          : 1;  //!< 3 CCDS: Capture/compare DMA selection
        mcu_reg_t mms           : 3;  //!< 6:4 MMS[2:0] Master mode selection
        mcu_reg_t ti1s          : 1;  //!< 7 TI1S: TI1 selection
        mcu_reg_t ois1          : 1;  //!< 8 OIS1: Output Idle state 1 (OC1 output)
        mcu_reg_t ois1n         : 1;  //!< 9 OIS1N: Output Idle state 1 (OC1N output)
        mcu_reg_t ois2          : 1;  //!< 10 OIS2: Output Idle state 2 (OC2 output)
        mcu_reg_t Reserved11_31 : 21; //!< 31:11 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx slave mode control register (TIMx_SMCR)
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tmr_smcr_t,
    {
        mcu_reg_t sms           : 3;  //!< 2:0 SMS[2:0] Slave mode selection
        mcu_reg_t Reserved1     : 1;  //!< 3 Reserved, must be kept at reset value
        mcu_reg_t ts            : 3;  //!< 6:4 TS[2:0] Trigger selection
        mcu_reg_t msm           : 1;  //!< 7 MSM: Master/Slave mode
        mcu_reg_t Reserved8_15  : 8;  //!< 15:8 Reserved, must be kept at reset value
        mcu_reg_t sms3          : 1;  //!< 16 SMS3 Slave mode selection bit 3
        mcu_reg_t Reserved17_31 : 15; //!< 31:15 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx DMA/interrupt enable register (TIMx_DIER)
 *        Address offset: 0x0C
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_dier_t,
    {
        mcu_reg_t uie           : 1;  //!< 0 UIE: Update interrupt enable
        mcu_reg_t cc1ie         : 1;  //!< 1 CC1IE: Capture/Compare 1 interrupt enable
        mcu_reg_t cc2ie         : 1;  //!< 2 CC2IE: Capture/Compare 2 interrupt enable
        mcu_reg_t Reserved3_4   : 2;  //!< 4:3 Reserved, must be kept at reset value
        mcu_reg_t comie         : 1;  //!< 5 COMIE: COM interrupt enable
        mcu_reg_t tie           : 1;  //!< 6 TIE: Trigger interrupt enable
        mcu_reg_t bie           : 1;  //!< 7 BIE: Break interrupt enable
        mcu_reg_t ude           : 1;  //!< 8 UDE: Update DMA request enable
        mcu_reg_t cc1de         : 1;  //!< 9 CC1DE: Capture/Compare 1 DMA request enable
        mcu_reg_t cc2de         : 1;  //!< 10 CC2DE: Capture/Compare 2 DMA request enable
        mcu_reg_t Reserved11_12 : 2;  //!< 12:11 Reserved, must be kept at reset value
        mcu_reg_t comde         : 1;  //!< 13 COMDE: COM DMA request enable
        mcu_reg_t tde           : 1;  //!< 14 TDE: Trigger DMA request enable
        mcu_reg_t Reserved15_31 : 17; //!< 31:15 Reserved, must be kept at reset value.
    }
);

/**
 *  @brief TIMx status register (TIMx_SR)
 *         Address offset: 0x10
 *         Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_sr_t,
    {
        mcu_reg_t uif           : 1;  //!< 0 UIF: Update interrupt flag
        mcu_reg_t cc1if         : 1;  //!< 1 CC1IF: Capture/Compare 1 interrupt flag
        mcu_reg_t cc2if         : 1;  //!< 2 CC2IF: Capture/Compare 2 interrupt flag
        mcu_reg_t Reserved3_4   : 2;  //!< 4:3 Reserved, must be kept at reset value
        mcu_reg_t comif         : 1;  //!< 5 COMIF: COM interrupt flag
        mcu_reg_t tif           : 1;  //!< 6 TIF: Trigger interrupt flag
        mcu_reg_t bif           : 1;  //!< 7 BIF: Break interrupt flag
        mcu_reg_t Reserved8     : 1;  //!< 8 Reserved, must be kept at reset value
        mcu_reg_t cc1of         : 1;  //!< 9 CC1OF: Capture/Compare 1 overcapture flag
        mcu_reg_t cc2of         : 1;  //!< 10 CC2OF: Capture/compare 2 overcapture flag
        mcu_reg_t Reserved11_31 : 21; //!< 31:11 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx event generation register (TIMx_EGR)
 *        Address offset: 0x14
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_egr_t,
    {
        mcu_reg_t ug           : 1;  //!< 0 UG: Update generation
        mcu_reg_t cc1g         : 1;  //!< 1 CC1G: Capture/compare 1 generation
        mcu_reg_t cc2g         : 1;  //!< 2 CC2G: Capture/compare 2 generation
        mcu_reg_t Reserved3_4  : 2;  //!< 4:3 Reserved, must be kept at reset value
        mcu_reg_t comg         : 1;  //!< 5 COMG: Capture/Compare control update generation
        mcu_reg_t tg           : 1;  //!< 6 TG: Trigger generation
        mcu_reg_t bg           : 1;  //!< 7 BG: Break generation
        mcu_reg_t Reserved8_31 : 24; //!< 31:8 Reserved, must be kept at reset value.
    }
);

/**
 * @brief TIMx capture/compare mode register 1 (TIMx_CCMR1)
 *        Address offset: 0x18
 *        Reset value: 0x0000
 *
 *        The same register can be used for input capture mode (this section) or for output compare
 *        mode (next section). The direction of a channel is defined by configuring the corresponding
 *        CCxS bits. All the other bits of this register have a different function in input and in output
 *        mode.
 */
typedef union
{
    mcu_reg_t d32;
    struct
    {// Output compare mode:
        mcu_reg_t cc1s          : 2;  //!< 1:0 CC1S[1:0] Capture/Compare 1 selection
        mcu_reg_t oc1fe         : 1;  //!< 2 OC1FE: Output compare 1 fast enable
        mcu_reg_t oc1pe         : 1;  //!< 3 OC1PE: Output compare 1 preload enable
        mcu_reg_t oc1m          : 3;  //!< 6:4 OC1M[2:0] Output compare 1 mode
        mcu_reg_t Reserved7     : 1;  //!< 7 Reserved, must be kept at reset value
        mcu_reg_t cc2s          : 2;  //!< 9:8 CC2S[1:0] Capture/Compare 2 selection
        mcu_reg_t oc2fe         : 1;  //!< 10 OC2FE: Output compare 2 fast enable
        mcu_reg_t oc2pe         : 1;  //!< 11 OC2PE: Output compare 2 preload enable
        mcu_reg_t oc2m          : 3;  //!< 14:12 OC2M[2:0] Output compare 2 mode
        mcu_reg_t Reserved15    : 1;  //!< 15 Reserved, must be kept at reset value
        mcu_reg_t oc1m3         : 1;  //!< 16 OC1M3 Output compare 1 mode, bit 3
        mcu_reg_t Reserved17_23 : 7;  //!< 23:17 Reserved, must be kept at reset value
        mcu_reg_t oc2m3         : 1;  //!< 24 OC2M3 Output compare 2 mode, bit 3
        mcu_reg_t Reserved25_31 : 7;  //!< 31:25 Reserved, must be kept at reset value
    } out;
    struct
    {// Input compare mode:
        mcu_reg_t cc1s          : 2;  //!< 1:0 CC1S[1:0] Capture/Compare 1 selection
        mcu_reg_t ic1psc        : 2;  //!< 3:2 IC1PSC[1:0]: Input capture 1 prescaler
        mcu_reg_t ic1f          : 4;  //!< 7:4 IC1F[3:0] Input capture 1 filter
        mcu_reg_t cc2s          : 2;  //!< 9:8 CC2S[1:0] Capture/compare 2 selection
        mcu_reg_t ic2psc        : 2;  //!< 11:10 IC2PSC[1:0] Input capture 2 prescaler
        mcu_reg_t ic2f          : 4;  //!< 15:12 IC2F[3:0] Input capture 2 filter
        mcu_reg_t Reserved16_31 : 16; //!< 31:16 Reserved, must be kept at reset value
    } in;
} tmr_ccmr1_t;
CHECK_REG_SIZE(tmr_ccmr1_t);

/**
 * @brief TIMx capture/compare mode register 1 (TIMx_CCMR1)
 *        Address offset: 0x1C
 *        Reset value: 0x0000
 *
 *        The same register can be used for input capture mode (this section) or for output compare
 *        mode (next section). The direction of a channel is defined by configuring the corresponding
 *        CCxS bits. All the other bits of this register have a different function in input and in output
 *        mode.
 */
typedef union
{
    mcu_reg_t d32;
    struct
    {// Output compare mode:
        mcu_reg_t cc3s          : 2;  //!< 1:0 CC3S[1:0] Capture/Compare 3 selection
        mcu_reg_t oc3fe         : 1;  //!< 2 OC3FE: Output compare 3 fast enable
        mcu_reg_t oc3pe         : 1;  //!< 3 OC3PE: Output compare 3 preload enable
        mcu_reg_t oc3m          : 3;  //!< 6:4 OC3M[2:0] Output compare 3 mode
        mcu_reg_t Reserved7     : 1;  //!< 7 Reserved, must be kept at reset value
        mcu_reg_t cc4s          : 2;  //!< 9:8 CC4S[1:0] Capture/Compare 4 selection
        mcu_reg_t oc4fe         : 1;  //!< 10 OC4FE: Output compare 4 fast enable
        mcu_reg_t oc4pe         : 1;  //!< 11 OC4PE: Output compare 4 preload enable
        mcu_reg_t oc4m          : 3;  //!< 14:12 OC4M[2:0] Output compare 4 mode
        mcu_reg_t Reserved15    : 1;  //!< 15 Reserved, must be kept at reset value
        mcu_reg_t oc3m3         : 1;  //!< 16 OC3M3 Output compare 3 mode, bit 3
        mcu_reg_t Reserved17_23 : 7;  //!< 23:17 Reserved, must be kept at reset value
        mcu_reg_t oc4m3         : 1;  //!< 24 OC4M3 Output compare 4 mode, bit 3
        mcu_reg_t Reserved25_31 : 7;  //!< 31:25 Reserved, must be kept at reset value
    } out;
    struct
    {// Input compare mode:
        mcu_reg_t cc3s          : 2;  //!< 1:0 CC3S[1:0] Capture/Compare 3 selection
        mcu_reg_t ic3psc        : 2;  //!< 3:2 IC3PSC[1:0]: Input capture 3 prescaler
        mcu_reg_t ic3f          : 4;  //!< 7:4 IC3F[3:0] Input capture 3 filter
        mcu_reg_t cc4s          : 2;  //!< 9:8 CC4S[1:0] Capture/compare 4 selection
        mcu_reg_t ic4psc        : 2;  //!< 11:10 IC4PSC[1:0] Input capture 4 prescaler
        mcu_reg_t ic4f          : 4;  //!< 15:12 IC4F[3:0] Input capture 4 filter
        mcu_reg_t Reserved16_31 : 16; //!< 31:16 Reserved, must be kept at reset value
    } in;
} tmr_ccmr2_t;
CHECK_REG_SIZE(tmr_ccmr2_t);

/**
 * @brief TIMx capture/compare enable register (TIMx_CCER)
 *        Address offset: 0x20
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_ccer_t,
    {
        mcu_reg_t cc1e          : 1;  //!< 0 CC1E: Capture/Compare 1 output enable
        mcu_reg_t cc1p          : 1;  //!< 1 CC1P: Capture/Compare 1 output polarity
        mcu_reg_t cc1ne         : 1;  //!< 2 CC1NE: Capture/Compare 1 complementary output enable
        mcu_reg_t cc1np         : 1;  //!< 3 CC1NP: Capture/Compare 1 complementary output polarity
        mcu_reg_t cc2e          : 1;  //!< 4 CC2E: Capture/Compare 2 output enable
        mcu_reg_t cc2p          : 1;  //!< 5 CC2P: Capture/Compare 2 output polarity
        mcu_reg_t Reserved6     : 1;  //!< 6 Reserved, must be kept at reset value
        mcu_reg_t cc2np         : 1;  //!< 7 CC2NP: Capture/Compare 2 complementary output polarity
        mcu_reg_t Reserved8_31  : 24; //!< 31:8 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx counter (TIMx_CNT)
 *        Address offset: 0x24
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tmr_cnt_t,
    {
        mcu_reg_t cnt           : 32; //!< 31:0 CNT[31:0]: Counter value
    }
);

/**
 *  @brief TIMx prescaler (TIMx_PSC)
 *         Address offset: 0x28
 *         Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_psc_t,
    {
        mcu_reg_t psc           : 16; //!< 15:0 PSC[15:0]: Prescaler value
        mcu_reg_t Reserved16_31 : 16; //!< 31:16 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx auto-reload register (TIMx_ARR)
 *        Address offset: 0x2C
 *        Reset value: 0xFFFF
 */
MCU_REG_TYPE (
    tmr_arr_t,
    {
        mcu_reg_t arr           : 32; //!< 31:0 ARR[31:0]: Auto-reload value
    }
);

/**
 * @brief TIMx repetition counter register (TIMx_RCR)
 *        Address offset: 0x30
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_rcr_t,
    {
        mcu_reg_t rep          : 8;  //!< 7:0 REP[7:0]: Repetition counter value
        mcu_reg_t Reserved8_31 : 24; //!< 31:8 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx capture/compare register (TIMx_CCRx)
 *        Address offset: 0x34
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    tmr_ccrX_t,
    {
        mcu_reg_t ccr           : 32; //!< 31:0 CCRX[31:0]: Capture/Compare value
    }
);

/**
 * @brief TIMx break and dead-time register (TIMx_BDTR)
 *        Address offset: 0x44
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_bdtr_t,
    {
        mcu_reg_t dt            : 8;  //!< 7:0 DTG[7:0]: Dead-time generator setup
        mcu_reg_t lock          : 2;  //!< 9:8 LOCK[1:0]: Lock configuration
        mcu_reg_t ossi          : 1;  //!< 10 OSSI: Off-state selection for Idle mode
        mcu_reg_t ossr          : 1;  //!< 11 OSSR: Off-state selection for Run mode
        mcu_reg_t bke           : 1;  //!< 12 BKE: Break enable
        mcu_reg_t bkp           : 1;  //!< 13 BKP: Break polarity
        mcu_reg_t aoe           : 1;  //!< 14 AOE: Automatic output enable
        mcu_reg_t moe           : 1;  //!< 15 MOE: Main output enable
        mcu_reg_t Reserved16_31 : 16; //!< 31:16 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx DMA control register (TIMx_DCR)
 *        Address offset: 0x48
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_dcr_t,
    {
        mcu_reg_t dba           : 5;  //!< 4:0 DBA[4:0] DMA base address
        mcu_reg_t Reserved5_7   : 3;  //!< 7:5 Reserved, must be kept at reset value
        mcu_reg_t dbl           : 5;  //!< 12:8 DBL[4:0] DMA burst length
        mcu_reg_t Reserved13_31 : 19; //!< 31:13 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx DMA address for full transfer (TIMx_DMAR)
 *        Address offset: 0x4C
 *        Reset value: 0x0000
 */
MCU_REG_TYPE (
    tmr_dmar_t,
    {
        mcu_reg_t dmab          : 16; //!< 15:0 DMAB[15:0] DMA register for burst accesses
        mcu_reg_t Reserved16_31 : 16; //!< 31:16 Reserved, must be kept at reset value
    }
);

/**
 * @brief TIMx register map
 */
typedef struct
{
    volatile tmr_cr1_t   CR1;   //!< TIMx_CR1   : 0x0000
    volatile tmr_cr2_t   CR2;   //!< TIMx_CR2   : 0x0004
    volatile tmr_smcr_t  SMCR;  //!< TIMx_SMCR  : 0x0008
    volatile tmr_dier_t  DIER;  //!< TIMx_DIER  : 0x000C
    volatile tmr_sr_t    SR;    //!< TIMx_SR    : 0x0010
    volatile tmr_egr_t   EGR;   //!< TIMx_EGR   : 0x0014
    volatile tmr_ccmr1_t CCMR1; //!< TIMx_CCMR1 : 0x0018
    volatile tmr_ccmr2_t CCMR2; //!< TIMx_CCMR2 : 0x001C
    volatile tmr_ccer_t  CCER;  //!< TIMx_CCER  : 0x0020
    volatile tmr_cnt_t   CNT;   //!< TIMx_CNT   : 0x0024
    volatile tmr_psc_t   PSC;   //!< TIMx_PSC   : 0x0028
    volatile tmr_arr_t   ARR;   //!< TIMx_ARR   : 0x002C
    volatile tmr_rcr_t   RCR;   //!< TIMx_RCR   : 0x0030
    volatile tmr_ccrX_t  CCR1;  //!< TIMx_CCR1  : 0x0034
    volatile tmr_ccrX_t  CCR2;  //!< TIMx_CCR2  : 0x0038
    volatile tmr_ccrX_t  CCR3;  //!< TIMx_CCR3  : 0x003C
    volatile tmr_ccrX_t  CCR4;  //!< TIMx_CCR4  : 0x0040
    volatile tmr_bdtr_t  BDTR;  //!< TIMx_BDTR  : 0x0044
    volatile tmr_dcr_t   DCR;   //!< TIMx_DCR   : 0x0048
    volatile tmr_dmar_t  DMAR;  //!< TIMx_DMAR  : 0x004C
} tmr_regs_t;

CHECK_REG_MAP(tmr_regs_t, 20);

#define TMR1_REG_ADDR  ((tmr_regs_t *)TIM1_BASE)
#define TMR2_REG_ADDR  ((tmr_regs_t *)TIM2_BASE)
#define TMR3_REG_ADDR  ((tmr_regs_t *)TIM3_BASE)
#define TMR4_REG_ADDR  ((tmr_regs_t *)TIM4_BASE)
#define TMR5_REG_ADDR  ((tmr_regs_t *)TIM5_BASE)
#define TMR6_REG_ADDR  ((tmr_regs_t *)TIM6_BASE)
#define TMR7_REG_ADDR  ((tmr_regs_t *)TIM7_BASE)
#define TMR8_REG_ADDR  ((tmr_regs_t *)TIM8_BASE)
#define TMR15_REG_ADDR ((tmr_regs_t *)TIM15_BASE)
#define TMR16_REG_ADDR ((tmr_regs_t *)TIM16_BASE)
#define TMR17_REG_ADDR ((tmr_regs_t *)TIM17_BASE)

#endif  // STM32L475_TMR_REG_H
