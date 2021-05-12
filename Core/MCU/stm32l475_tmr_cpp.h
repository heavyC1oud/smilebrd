/**
 * @file stm32l475_tmr_cpp.h
 *
 *       Timer
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_TMR_CPP_H
#define STM32L475_TMR_CPP_H

#include <cstdint>
#include "regs.h"
#include "stm32l4xx.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_tmr_reg.h"


/**
 * @brief Список возможных таймеров
 */
typedef enum
{
    TMR_1,
    TMR_2,
    TMR_3,
    TMR_4,
    TMR_5,
    TMR_6,
    TMR_7,
    TMR_8,
    TMR_15,
    TMR_16,
    TMR_17,
} tmr_num_t;

/**
 * @brief Список доступных каналов
 */
typedef enum
{
    TMR_CH1 = 0,
    TMR_CH2 = 1,
    TMR_CH3 = 2,
    TMR_CH4 = 3,
} tmr_channel_t;

/**
 * @brief Список возможных режимов канала
 *        Данный контроллер имеет больше режимов работы, но они здесь не приводятся.
 *        Если есть необходимость добавить новые режимы, то нужно добавить установку
 *        бита 'CCMRx.ocxm_3' в функции 'set_compare_mode', где в данный момент
 *        он не учитывается.
 */
typedef enum
{
    TMR_CMP_FROZEN      = 0,    /*!< The comparison between the output compare register TIMx_CCR1
                                     and the counter TIMx_CNT has no effect on the outputs. */
    TMR_CMP_CH1_ACT     = 1,    /*!< Set channel 1 to active level on match. OC1REF signal is forced
                                     high when the counter TIMx_CNT matches the capture/compare
                                     register 1 (TIMx_CCR1).*/
    TMR_CMP_CH1_INACT   = 2,    /*!< Set channel 1 to inactive level on match. OC1REF signal is forced
                                     low when the counter TIMx_CNT matches the capture/compare register
                                     1 (TIMx_CCR1). */
    TMR_CMP_TOGGLE      = 3,    /*!< Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.*/
    TMR_CMP_FORCE_INACT = 4,    /*!< Force inactive level - OC1REF is forced low.*/
    TMR_CMP_FORCE_ACT   = 5,    /*!< Force active level - OC1REF is forced high.*/
    TMR_CMP_PWM1        = 6,    /*!< PWM mode 1 - In upcounting, channel 1 is active as long as
                                     TIMx_CNT<TIMx_CCR1 else inactive. In downcounting, channel 1
                                     is inactive (OC1REF=‘0’) as long as TIMx_CNT>TIMx_CCR1 else
                                     active (OC1REF=’1’). */
    TMR_CMP_PWM2        = 7,    /*!< PWM mode 2 - In upcounting, channel 1 is inactive as long as
                                     TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1
                                     is active as long as TIMx_CNT>TIMx_CCR1 else inactive. */
} tmr_compare_t;

/**
 * @brief Список возможных направлений канала
 */
typedef enum
{
    TMR_CAP_DIR_OUT     = 0,
    TMR_CAP_DIR_IN_T1   = 1,
    TMR_CAP_DIR_IN_T2   = 2,
    TMR_CAP_DIR_IN_TRS  = 3,
} tmr_cap_dir_t;

template <tmr_num_t tmr_num>
class Tmr
{
private:
    static struct tmr_base_reg
    {
        tmr_regs_t* operator-> ()
        {
            return
                tmr_num == TMR_1 ?  TMR1_REG_ADDR:
                tmr_num == TMR_2 ?  TMR2_REG_ADDR:
                tmr_num == TMR_3 ?  TMR3_REG_ADDR:
                tmr_num == TMR_4 ?  TMR4_REG_ADDR:
                tmr_num == TMR_5 ?  TMR5_REG_ADDR:
                tmr_num == TMR_6 ?  TMR6_REG_ADDR:
                tmr_num == TMR_7 ?  TMR7_REG_ADDR:
                tmr_num == TMR_8 ?  TMR8_REG_ADDR:
                tmr_num == TMR_15 ? TMR15_REG_ADDR:
                tmr_num == TMR_16 ? TMR16_REG_ADDR:
                                    TMR17_REG_ADDR;
        }
    } TMR_REGS;

    static struct tmr_base_irq
    {
        operator IRQn_Type()
        {
            return
                tmr_num == TMR_1?  TIM1_BRK_TIM15_IRQn:
                tmr_num == TMR_2?  TIM2_IRQn:
                tmr_num == TMR_3?  TIM3_IRQn:
                tmr_num == TMR_4?  TIM4_IRQn:
                tmr_num == TMR_5?  TIM5_IRQn:
                tmr_num == TMR_6?  TIM6_DAC_IRQn:
                tmr_num == TMR_7?  TIM7_IRQn:
                tmr_num == TMR_8?  TIM8_BRK_IRQn:
                tmr_num == TMR_15? TIM1_BRK_TIM15_IRQn:
                tmr_num == TMR_16? TIM1_UP_TIM16_IRQn:
                                   TIM1_TRG_COM_TIM17_IRQn;
        }
    } TMR_IRQn;

    inline uint32_t get_rcc_clock()
    {
        return rcc_t::get_clock(RCC_CLOCK_APB1);
    }

    inline void rcc_on()
    {
        tmr_num == TMR_1?  rcc_t::apb2_periph_on(RCC_APB2_PERIPH_TIM1):
        tmr_num == TMR_2?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM2):
        tmr_num == TMR_3?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM3):
        tmr_num == TMR_4?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM4):
        tmr_num == TMR_5?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM5):
        tmr_num == TMR_6?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM6):
        tmr_num == TMR_7?  rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_TIM7):
        tmr_num == TMR_8?  rcc_t::apb2_periph_on(RCC_APB2_PERIPH_TIM8):
        tmr_num == TMR_15? rcc_t::apb2_periph_on(RCC_APB2_PERIPH_TIM15):
        tmr_num == TMR_16? rcc_t::apb2_periph_on(RCC_APB2_PERIPH_TIM16):
                           rcc_t::apb2_periph_on(RCC_APB2_PERIPH_TIM17);
    }

    inline void rcc_off()
    {
        tmr_num == TMR_1?  rcc_t::apb2_periph_off(RCC_APB2_PERIPH_TIM1):
        tmr_num == TMR_2?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM2):
        tmr_num == TMR_3?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM3):
        tmr_num == TMR_4?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM4):
        tmr_num == TMR_5?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM5):
        tmr_num == TMR_6?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM6):
        tmr_num == TMR_7?  rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_TIM7):
        tmr_num == TMR_8?  rcc_t::apb2_periph_off(RCC_APB2_PERIPH_TIM8):
        tmr_num == TMR_15? rcc_t::apb2_periph_off(RCC_APB2_PERIPH_TIM15):
        tmr_num == TMR_16? rcc_t::apb2_periph_off(RCC_APB2_PERIPH_TIM16):
                           rcc_t::apb2_periph_off(RCC_APB2_PERIPH_TIM17);
    }

    inline void rcc_reset_on()
    {
        tmr_num == TMR_1?  rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_TIM1):
        tmr_num == TMR_2?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM2):
        tmr_num == TMR_3?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM3):
        tmr_num == TMR_4?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM4):
        tmr_num == TMR_5?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM5):
        tmr_num == TMR_6?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM6):
        tmr_num == TMR_7?  rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_TIM7):
        tmr_num == TMR_8?  rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_TIM8):
        tmr_num == TMR_15? rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_TIM15):
        tmr_num == TMR_16? rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_TIM16):
                           rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_TIM17);
    }

    inline void rcc_reset_off()
    {
        tmr_num == TMR_1?  rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_TIM1):
        tmr_num == TMR_2?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM2):
        tmr_num == TMR_3?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM3):
        tmr_num == TMR_4?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM4):
        tmr_num == TMR_5?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM5):
        tmr_num == TMR_6?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM6):
        tmr_num == TMR_7?  rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_TIM7):
        tmr_num == TMR_8?  rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_TIM8):
        tmr_num == TMR_15? rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_TIM15):
        tmr_num == TMR_16? rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_TIM16):
                           rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_TIM17);
    }

    inline bool is_rcc_on()
    {
        return
            tmr_num == TMR_1?  rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_TIM1):
            tmr_num == TMR_2?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM2):
            tmr_num == TMR_3?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM3):
            tmr_num == TMR_4?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM4):
            tmr_num == TMR_5?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM5):
            tmr_num == TMR_6?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM6):
            tmr_num == TMR_7?  rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_TIM7):
            tmr_num == TMR_8?  rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_TIM8):
            tmr_num == TMR_15? rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_TIM15):
            tmr_num == TMR_16? rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_TIM16):
                               rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_TIM17);
    }

public:
    Tmr()
    {
        NVIC_DisableIRQ(TMR_IRQn);
        if(this->is_rcc_on()) {
            this->reset();
            this->rcc_off();
        }
    }

    void reset()
    {
        this->rcc_reset_on();
        this->rcc_reset_off();
    }

    inline void set_freq(uint32_t freq)
    {
        uint32_t tmr_freq = this->get_rcc_clock();
        if(tmr_freq == rcc_t::get_clock(RCC_CLOCK_APB1)) {
            TMR_REGS->PSC.psc = ((tmr_freq / freq)) - 1;
            TMR_REGS->EGR.ug = 1;
        }
        else {
            TMR_REGS->PSC.psc = ((tmr_freq / (freq >> 1))) - 1;
            TMR_REGS->EGR.ug = 1;
        }
    }

    inline uint32_t get_freq()
    {
        uint32_t tmr_freq = this->get_rcc_clock();
        if(tmr_freq == rcc_t::get_clock(RCC_CLOCK_AHB)) {
            return (tmr_freq / (TMR_REGS->PSC.psc + 1));
        }
        else {
            return ((tmr_freq / (TMR_REGS->PSC.psc + 1)) << 1);
        }
    }

    inline void on() {TMR_REGS->CR1.cen = 1;}
    inline void off() {TMR_REGS->CR1.cen = 0;}

    inline bool is_on() {return (bool)(TMR_REGS->CR1.cen == 1);}

    inline void set_arr(uint16_t cnt) {TMR_REGS->ARR.arr = cnt;}

    inline void set_ccrx(tmr_channel_t channel, uint16_t data)
    {
        switch(channel) {
        case TMR_CH1:
            TMR_REGS->CCR1.ccr = data;
            break;
        case TMR_CH2:
            TMR_REGS->CCR2.ccr = data;
            break;
        case TMR_CH3:
            TMR_REGS->CCR3.ccr = data;
            break;
        case TMR_CH4:
            TMR_REGS->CCR4.ccr = data;
            break;
        default:
            break;
        }
    }

    inline void set_direction_down() {TMR_REGS->CR1.dir = 1;}
    inline void set_direction_up() {TMR_REGS->CR1.dir = 0;}

    inline void set_counter(uint16_t cnt) {TMR_REGS->CNT.cnt = cnt;}
    inline uint16_t get_counter() {return TMR_REGS->CNT.cnt;}

    inline void set_prescale(uint16_t psc)
    {
        TMR_REGS->PSC.psc = psc;
        TMR_REGS->EGR.ug = 1;
    }

    inline uint16_t get_prescale() {return TMR_REGS->PSC.psc;}

    inline void enable_irq() {TMR_REGS->DIER.uie = 1;}
    inline void disable_irq() {TMR_REGS->DIER.uie = 0;}

    inline bool is_update() {return (bool)(TMR_REGS->SR.uif == 1);}

    inline void enable_dma_update() {TMR_REGS->DIER.ude = 1;}
    inline void disable_dma_update() {TMR_REGS->DIER.ude = 0;}

    inline void init()
    {
        if (this->is_rcc_on() == false) {
            this->rcc_on();
            this->reset();
        }
        NVIC_SetPriority(TMR_IRQn, 0x00);
        NVIC_EnableIRQ(TMR_IRQn);
    }

    inline uint32_t get_status() {return TMR_REGS->SR.d32;}
    inline void set_status(uint32_t data) {TMR_REGS->SR.d32 = data;}

    inline void set_direction(tmr_channel_t channel, tmr_cap_dir_t dir)
    {
        switch(channel) {
        case TMR_CH1:
            TMR_REGS->CCMR1.out.cc1s = dir;
            break;
        case TMR_CH2:
            TMR_REGS->CCMR1.out.cc2s = dir;
            break;
        case TMR_CH3:
            TMR_REGS->CCMR2.out.cc3s = dir;
            break;
        case TMR_CH4:
            TMR_REGS->CCMR2.out.cc4s = dir;
            break;
        default:
            break;
        }
    }

    inline void set_compare_mode(tmr_channel_t channel, tmr_compare_t mode)
    {
        switch(channel) {
        case TMR_CH1:
            TMR_REGS->CCMR1.out.oc1m = mode;
            break;
        case TMR_CH2:
            TMR_REGS->CCMR1.out.oc2m = mode;
            break;
        case TMR_CH3:
            TMR_REGS->CCMR2.out.oc3m = mode;
            break;
        case TMR_CH4:
            TMR_REGS->CCMR2.out.oc4m = mode;
            break;
        default:
            break;
        }
    }

    inline void on_capture(tmr_channel_t channel)
    {
        // Включает только прямые выходы
        TMR_REGS->CCER.d32 |= (1 << (channel << 2));
    }

    inline void off_capture(tmr_channel_t channel)
    {
        // Выключает только прямые выходы
        TMR_REGS->CCER.d32 &= ~(1 << (channel << 2));
    }

    // set output polarity active low
    inline void active_low(tmr_channel_t channel)
    {
        TMR_REGS->CCER.d32 |= (1 << ((channel << 2) + 1));
    }

    // set output polarity active high
    inline void active_high(tmr_channel_t channel)
    {
        TMR_REGS->CCER.d32 &= ~(1 << ((channel << 2) + 1));
    }

    // set main output enable bit
    inline void enable_moe()
    {
        TMR_REGS->BDTR.moe = 1;
    }

    // clear main output enable bit
    inline void disable_moe()
    {
        TMR_REGS->BDTR.moe = 0;
    }
};

#endif  // STM32L475_TMR_CPP_H
