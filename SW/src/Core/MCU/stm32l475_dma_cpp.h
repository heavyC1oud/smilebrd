/**
 * @file stm32l475_dma_cpp.h
 *
 *       Direct memory access controller
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_DMA_CPP_H
#define STM32L475_DMA_CPP_H

#include "basis.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_dma_reg.h"


typedef enum
{
    DMA_1, DMA_2
} dma_num_t;

typedef enum
{
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
} dma_channel_no_t;

typedef enum
{
    DMA_DIR_PERIPH2MEM  = 0, /*!< Peripheral-to-memory */
    DMA_DIR_MEM2PERIPH  = 1, /*!< Memory-to-peripheral */
} dma_direction_t;

typedef enum
{
    DMA_PRIOR_LOW       = 0, /*!< Low */
    DMA_PRIOR_MEDIUM    = 1, /*!< Medium */
    DMA_PRIOR_HIGH      = 2, /*!< High */
    DMA_PRIOR_VERYHIGH  = 3, /*!< Very high */
} dma_priority_t;

typedef enum
{
    DMA_DATASIZE_8      = 0, /*!< byte (8-bit) */
    DMA_DATASIZE_16     = 1, /*!< half-word (16-bit) */
    DMA_DATASIZE_32     = 2, /*!< word (32-bit) */
} dma_datasize_t;

typedef enum
{
    DMA1_CHANNEL1_0_ADC1            = 0,
    DMA1_CHANNEL1_4_TIM3_CH3        = 4,
    DMA1_CHANNEL1_5_TIM17_CH1       = 5,
    DMA1_CHANNEL1_5_TIM17_UP        = 5,
    DMA1_CHANNEL1_6_TIM4_CH1        = 6,

    DMA1_CHANNEL2_0_ADC2            = 0,
    DMA1_CHANNEL2_1_SPI1_RX         = 1,
    DMA1_CHANNEL2_2_USART3_TX       = 2,
    DMA1_CHANNEL2_3_I2C3_TX         = 3,
    DMA1_CHANNEL2_4_TIM2_UP         = 4,
    DMA1_CHANNEL2_5_TIM3_CH3        = 5,
    DMA1_CHANNEL2_7_TIM1_CH1        = 7,

    DMA1_CHANNEL3_0_ADC3            = 0,
    DMA1_CHANNEL3_1_SPI1_TX         = 1,
    DMA1_CHANNEL3_2_USART3_RX       = 2,
    DMA1_CHANNEL3_3_I2C3_RX         = 3,
    DMA1_CHANNEL3_4_TIM16_CH1       = 4,
    DMA1_CHANNEL3_4_TIM16_UP        = 4,
    DMA1_CHANNEL3_5_TIM3_CH4        = 5,
    DMA1_CHANNEL3_5_TIM3_UP         = 5,
    DMA1_CHANNEL3_6_TIM6_UP         = 6,
    DMA1_CHANNEL3_6_DAC_CH1         = 6,
    DMA1_CHANNEL3_7_TIM1_CH2        = 7,

    DMA1_CHANNEL4_0_DFSDM1_FLT0     = 0,
    DMA1_CHANNEL4_1_SPI2_RX         = 1,
    DMA1_CHANNEL4_2_USART1_TX       = 2,
    DMA1_CHANNEL4_3_I2C2_TX         = 3,
    DMA1_CHANNEL4_5_TIM7_UP         = 5,
    DMA1_CHANNEL4_5_DAC_CH2         = 5,
    DMA1_CHANNEL4_6_TIM4_CH2        = 6,
    DMA1_CHANNEL4_7_TIM1_CH4        = 7,
    DMA1_CHANNEL4_7_TIM1_TRIG       = 7,
    DMA1_CHANNEL4_7_TIM1_COM        = 7,

    DMA1_CHANNEL5_0_DFSDM1_FLT1     = 0,
    DMA1_CHANNEL5_1_SPI2_TX         = 1,
    DMA1_CHANNEL5_2_USART1_RX       = 2,
    DMA1_CHANNEL5_3_I2C2_RX         = 3,
    DMA1_CHANNEL5_4_TIM2_CH1        = 4,
    DMA1_CHANNEL5_5_QUADSPI         = 5,
    DMA1_CHANNEL5_6_TIM4_CH3        = 6,
    DMA1_CHANNEL5_7_TIM15_CH1       = 7,
    DMA1_CHANNEL5_7_TIM15_UP        = 7,
    DMA1_CHANNEL5_7_TIM15_TRIG      = 7,
    DMA1_CHANNEL5_7_TIM15_COM       = 7,

    DMA1_CHANNEL6_0_DFSDM1_FLT2     = 0,
    DMA1_CHANNEL6_1_SAI2_A          = 1,
    DMA1_CHANNEL6_2_USART2_RX       = 2,
    DMA1_CHANNEL6_3_I2C1_TX         = 3,
    DMA1_CHANNEL6_4_TIM16_CH1       = 4,
    DMA1_CHANNEL6_4_TIM16_UP        = 4,
    DMA1_CHANNEL6_5_TIM3_CH1        = 5,
    DMA1_CHANNEL6_5_TIM3_TRIG       = 5,
    DMA1_CHANNEL6_7_TIM1_UP         = 7,

    DMA1_CHANNEL7_0_DFSDM1_FLT3     = 0,
    DMA1_CHANNEL7_1_SAI2_B          = 1,
    DMA1_CHANNEL7_2_USART2_TX       = 2,
    DMA1_CHANNEL7_3_I2C1_RX         = 3,
    DMA1_CHANNEL7_4_TIM2_CH2        = 4,
    DMA1_CHANNEL7_4_TIM2_CH4        = 4,
    DMA1_CHANNEL7_5_TIM17_CH1       = 5,
    DMA1_CHANNEL7_5_TIM17_UP        = 5,
    DMA1_CHANNEL7_6_TIM4_UP         = 6,
    DMA1_CHANNEL7_7_TIM1_CH3        = 7,


    DMA2_CHANNEL1_0_I2C4_RX         = 0,
    DMA2_CHANNEL1_1_SAI1_A          = 1,
    DMA2_CHANNEL1_2_UART5_TX        = 2,
    DMA2_CHANNEL1_3_SPI3_RX         = 3,
    DMA2_CHANNEL1_4_SWPMI1_RX       = 4,
    DMA2_CHANNEL1_5_TIM5_CH4        = 5,
    DMA2_CHANNEL1_5_TIM5_TRIG       = 5,
    DMA2_CHANNEL1_6_AES_IN          = 6,
    DMA2_CHANNEL1_7_TIM8_CH3        = 7,
    DMA2_CHANNEL1_7_TIM8_UP         = 7,

    DMA2_CHANNEL2_0_I2C4_TX         = 0,
    DMA2_CHANNEL2_1_SAI1_B          = 1,
    DMA2_CHANNEL2_2_UART5_RX        = 2,
    DMA2_CHANNEL2_3_SPI3_TX         = 3,
    DMA2_CHANNEL2_4_SWPMI1_TX       = 4,
    DMA2_CHANNEL2_5_TIM5_CH3        = 5,
    DMA2_CHANNEL2_5_TIM5_UP         = 5,
    DMA2_CHANNEL2_6_AES_OUT         = 6,
    DMA2_CHANNEL2_7_TIM8_CH4        = 7,
    DMA2_CHANNEL2_7_TIM8_TRIG       = 7,
    DMA2_CHANNEL2_7_TIM8_COM        = 7,

    DMA2_CHANNEL3_0_ADC1            = 0,
    DMA2_CHANNEL3_1_SAI2_A          = 1,
    DMA2_CHANNEL3_2_UART4_TX        = 2,
    DMA2_CHANNEL3_4_SPI1_RX         = 4,
    DMA2_CHANNEL3_6_AES_OUT         = 6,

    DMA2_CHANNEL4_0_ADC2            = 0,
    DMA2_CHANNEL4_1_SAI2_B          = 1,
    DMA2_CHANNEL4_3_TIM6_UP         = 3,
    DMA2_CHANNEL4_3_DAC_CH1         = 3,
    DMA2_CHANNEL4_4_SPI1_TX         = 4,
    DMA2_CHANNEL4_5_TIM5_CH2        = 5,
    DMA2_CHANNEL4_7_SDMMC1          = 7,

    DMA2_CHANNEL5_0_ADC3            = 0,
    DMA2_CHANNEL5_2_UART4_RX        = 2,
    DMA2_CHANNEL5_3_TIM7_UP         = 3,
    DMA2_CHANNEL5_3_DAC_CH2         = 3,
    DMA2_CHANNEL5_4_DCMI            = 4,
    DMA2_CHANNEL5_5_TIM5_CH1        = 5,
    DMA2_CHANNEL5_6_AES_IN          = 6,
    DMA2_CHANNEL5_7_SDMMC1          = 7,

    DMA2_CHANNEL6_0_DCMI            = 0,
    DMA2_CHANNEL6_1_SAI1_A          = 1,
    DMA2_CHANNEL6_2_USART1_TX       = 2,
    DMA2_CHANNEL6_4_LPUART1_TX      = 4,
    DMA2_CHANNEL6_5_I2C1_RX         = 5,
    DMA2_CHANNEL6_7_TIM8_CH1        = 7,

    DMA2_CHANNEL7_1_SAI1B           = 1,
    DMA2_CHANNEL7_2_USART1_RX       = 2,
    DMA2_CHANNEL7_3_QUADSPI         = 3,
    DMA2_CHANNEL7_4_LPUART1_RX      = 4,
    DMA2_CHANNEL7_5_I2C1_TX         = 5,
    DMA2_CHANNEL7_6_HASH_IN         = 6,
    DMA2_CHANNEL7_7_TIM8_CH2        = 7,
} dma_channel_t;

template<dma_num_t num, dma_channel_no_t channel_no>
struct dma_t
{
private:
    static struct base_dma_regs
    {
        dma_regs_t* operator-> ()
        {
            return
                num == DMA_1 ? DMA1_REGS_ADDR:
                               DMA2_REGS_ADDR;
        }
    } DMA_REGS;

    static struct base_dma_channel_regs
    {
        dma_regs_channel_t* operator-> ()
        {
            return &DMA_REGS->CHANNELS[channel_no];
        }
    } DMA_CHANNEL_REGS;

    static struct base_dma_irq
    {
        operator IRQn_Type()
        {
            return
                num == DMA_1 ?
                    channel_no == CHANNEL_1 ? DMA1_Channel1_IRQn:
                    channel_no == CHANNEL_2 ? DMA1_Channel2_IRQn:
                    channel_no == CHANNEL_3 ? DMA1_Channel3_IRQn:
                    channel_no == CHANNEL_4 ? DMA1_Channel4_IRQn:
                    channel_no == CHANNEL_5 ? DMA1_Channel5_IRQn:
                    channel_no == CHANNEL_6 ? DMA1_Channel6_IRQn:
                                            DMA1_Channel7_IRQn:
                num == DMA_2 ?
                    channel_no == CHANNEL_1 ? DMA2_Channel1_IRQn:
                    channel_no == CHANNEL_2 ? DMA2_Channel2_IRQn:
                    channel_no == CHANNEL_3 ? DMA2_Channel3_IRQn:
                    channel_no == CHANNEL_4 ? DMA2_Channel4_IRQn:
                    channel_no == CHANNEL_5 ? DMA2_Channel5_IRQn:
                    channel_no == CHANNEL_6 ? DMA2_Channel6_IRQn:
                                            DMA2_Channel7_IRQn:
                                            DMA2_Channel7_IRQn;
        }
    } DMAx_IRQn;

    uint32_t get_rcc_clock()
    {
        return rcc_t::get_clock(RCC_CLOCK_AHB);
    };

    void rcc_on()
    {
        num == DMA_1 ? rcc_t::ahb1_periph_on(RCC_AHB1_PERIPH_DMA1):
                       rcc_t::ahb1_periph_on(RCC_AHB1_PERIPH_DMA2);
    };

    void rcc_off()
    {
        num == DMA_1 ? rcc_t::ahb1_periph_off(RCC_AHB1_PERIPH_DMA1):
                       rcc_t::ahb1_periph_off(RCC_AHB1_PERIPH_DMA2);
    };

    void rcc_reset_on()
    {
        num == DMA_1 ? rcc_t::ahb1_periph_reset_on(RCC_AHB1_PERIPH_DMA1):
                       rcc_t::ahb1_periph_reset_on(RCC_AHB1_PERIPH_DMA2);
    };

    void rcc_reset_off()
    {
        num == DMA_1 ? rcc_t::ahb1_periph_reset_off(RCC_AHB1_PERIPH_DMA1):
                       rcc_t::ahb1_periph_reset_off(RCC_AHB1_PERIPH_DMA2);
    };

    bool is_rcc_on()
    {
        return
                num == DMA_1 ? rcc_t::is_ahb1_periph_on(RCC_AHB1_PERIPH_DMA1):
                               rcc_t::is_ahb1_periph_on(RCC_AHB1_PERIPH_DMA2);
    }

public:
    void reset()
    {
        this->rcc_reset_on();
        this->rcc_reset_off();
    };

    dma_t()
    {
        if(this->is_rcc_on()) {
            this->reset();
            this->rcc_off();
        }
        NVIC_DisableIRQ(DMAx_IRQn);
    }
    uint32_t get_dma_addr() {return (uint32_t)&(DMA_REGS->ISR);}
    uint32_t get_channel_addr() {return (uint32_t)&(DMA_CHANNEL_REGS->CCR);}

    /**
     * @brief Инициализация DMA
     *
     * @param channel - тип канала передачи
     * @param dir - направление передачи
     * @param circ - использовать кольцевой буффер
     * @param prior - приоритет
     * @param mem_size - размерность данных в памяти
     * @param meminc - инкрементация адресов в памяти
     * @param periph_size - размерность данных в периферии
     * @param periphinc - инкрементация адресов в периферии
     */
    void init(
            dma_channel_t channel,
            dma_direction_t dir,
            onoff_t circ,
            dma_priority_t prior,
            dma_datasize_t mem_size,
            onoff_t meminc,
            dma_datasize_t periph_size,
            onoff_t periphinc
        )
    {
        if(this->is_rcc_on() == false) {
            this->rcc_on();
            this->reset();
        }

        if(this->is_on() == false) {
            this->set_channel(channel);
            this->direction(dir);
            this->circ_mode(circ);
            this->priority(prior);
            this->memory_size(mem_size);
            this->inc_mem(meminc);
            this->periph_size(periph_size);
            this->inc_periph(periphinc);
            NVIC_SetPriority(DMAx_IRQn, 0x00);
            NVIC_EnableIRQ(DMAx_IRQn);
        }
    }

    inline bool is_on() {return (bool)( DMA_CHANNEL_REGS->CCR.en == 1);}

    inline void set_channel(dma_channel_t channel) { DMA_REGS->CSELR.d32 |= channel << (channel_no * 4);}

    inline void direction(dma_direction_t dir) { DMA_CHANNEL_REGS->CCR.dir = dir;}

    inline void circ_mode(onoff_t fl) { DMA_CHANNEL_REGS->CCR.circ = fl;}

    inline void priority(dma_priority_t pl) { DMA_CHANNEL_REGS->CCR.pl = pl;}

    inline void memory_size(dma_datasize_t sz) { DMA_CHANNEL_REGS->CCR.msize = sz;}

    inline void inc_mem(onoff_t fl) { DMA_CHANNEL_REGS->CCR.minc = fl;}

    inline void periph_size(dma_datasize_t sz) { DMA_CHANNEL_REGS->CCR.psize = sz;}

    inline void inc_periph(onoff_t fl) { DMA_CHANNEL_REGS->CCR.pinc = fl;}

    inline void on() { DMA_CHANNEL_REGS->CCR.en = 1;}

    inline void off() { DMA_CHANNEL_REGS->CCR.en = 0;}

    inline void set_ccr(uint32_t data) { DMA_CHANNEL_REGS->CCR.d32 = data;}

    inline void data_size(uint16_t data) { DMA_CHANNEL_REGS->CNDTR.ndt = data;}

    inline void periph_addr(uint32_t data) { DMA_CHANNEL_REGS->CPAR.pa = data;}

    inline void memory_addr(uint32_t data) { DMA_CHANNEL_REGS->CMAR.ma = data;}

    inline uint32_t get_memory_addr() { return DMA_CHANNEL_REGS->CMAR.ma;}

    inline void tc_irq_en() { DMA_CHANNEL_REGS->CCR.tcie = 1;}
    inline void tc_irq_dis() { DMA_CHANNEL_REGS->CCR.tcie = 0;}

    inline void ht_irq_en() { DMA_CHANNEL_REGS->CCR.htie = 1;}
    inline void ht_irq_dis() { DMA_CHANNEL_REGS->CCR.htie = 0;}

    inline void te_irq_en() { DMA_CHANNEL_REGS->CCR.teie = 1;}
    inline void te_irq_dis() { DMA_CHANNEL_REGS->CCR.teie = 0;}

    inline bool is_te_irq()
    {
        return
            channel_no == CHANNEL_1 ? (DMA_REGS->ISR.teif1 == 1):
            channel_no == CHANNEL_2 ? (DMA_REGS->ISR.teif2 == 1):
            channel_no == CHANNEL_3 ? (DMA_REGS->ISR.teif3 == 1):
            channel_no == CHANNEL_4 ? (DMA_REGS->ISR.teif4 == 1):
            channel_no == CHANNEL_5 ? (DMA_REGS->ISR.teif5 == 1):
            channel_no == CHANNEL_6 ? (DMA_REGS->ISR.teif6 == 1):
                                    (DMA_REGS->ISR.teif7 == 1);
    }

    inline bool is_ht_irq()
    {
        return
            channel_no == CHANNEL_1 ? (DMA_REGS->ISR.htif1 == 1):
            channel_no == CHANNEL_2 ? (DMA_REGS->ISR.htif2 == 1):
            channel_no == CHANNEL_3 ? (DMA_REGS->ISR.htif3 == 1):
            channel_no == CHANNEL_4 ? (DMA_REGS->ISR.htif4 == 1):
            channel_no == CHANNEL_5 ? (DMA_REGS->ISR.htif5 == 1):
            channel_no == CHANNEL_6 ? (DMA_REGS->ISR.htif6 == 1):
                                    (DMA_REGS->ISR.htif7 == 1);
    }

    inline bool is_tc_irq()
    {
        return
            channel_no == CHANNEL_1 ? (DMA_REGS->ISR.tcif1 == 1):
            channel_no == CHANNEL_2 ? (DMA_REGS->ISR.tcif2 == 1):
            channel_no == CHANNEL_3 ? (DMA_REGS->ISR.tcif3 == 1):
            channel_no == CHANNEL_4 ? (DMA_REGS->ISR.tcif4 == 1):
            channel_no == CHANNEL_5 ? (DMA_REGS->ISR.tcif5 == 1):
            channel_no == CHANNEL_6 ? (DMA_REGS->ISR.tcif6 == 1):
                                    (DMA_REGS->ISR.tcif7 == 1);
    }

    inline bool is_all_irq()
    {
        return
            channel_no == CHANNEL_1 ? (DMA_REGS->ISR.gif1 == 1):
            channel_no == CHANNEL_2 ? (DMA_REGS->ISR.gif2 == 1):
            channel_no == CHANNEL_3 ? (DMA_REGS->ISR.gif3 == 1):
            channel_no == CHANNEL_4 ? (DMA_REGS->ISR.gif4 == 1):
            channel_no == CHANNEL_5 ? (DMA_REGS->ISR.gif5 == 1):
            channel_no == CHANNEL_6 ? (DMA_REGS->ISR.gif6 == 1):
                                    (DMA_REGS->ISR.gif7 == 1);
    }

    inline void te_irq_clear()
    {
        channel_no == CHANNEL_1 ? (DMA_REGS->IFCR.cteif1 = 1):
        channel_no == CHANNEL_2 ? (DMA_REGS->IFCR.cteif2 = 1):
        channel_no == CHANNEL_3 ? (DMA_REGS->IFCR.cteif3 = 1):
        channel_no == CHANNEL_4 ? (DMA_REGS->IFCR.cteif4 = 1):
        channel_no == CHANNEL_5 ? (DMA_REGS->IFCR.cteif5 = 1):
        channel_no == CHANNEL_6 ? (DMA_REGS->IFCR.cteif6 = 1):
                                (DMA_REGS->IFCR.cteif7 = 1);
    }

    inline void ht_irq_clear()
    {
        channel_no == CHANNEL_1 ? (DMA_REGS->IFCR.chtif1 = 1):
        channel_no == CHANNEL_2 ? (DMA_REGS->IFCR.chtif2 = 1):
        channel_no == CHANNEL_3 ? (DMA_REGS->IFCR.chtif3 = 1):
        channel_no == CHANNEL_4 ? (DMA_REGS->IFCR.chtif4 = 1):
        channel_no == CHANNEL_5 ? (DMA_REGS->IFCR.chtif5 = 1):
        channel_no == CHANNEL_6 ? (DMA_REGS->IFCR.chtif6 = 1):
                                (DMA_REGS->IFCR.chtif7 = 1);
    }

    inline void tc_irq_clear()
    {
        channel_no == CHANNEL_1 ? (DMA_REGS->IFCR.ctcif1 = 1):
        channel_no == CHANNEL_2 ? (DMA_REGS->IFCR.ctcif2 = 1):
        channel_no == CHANNEL_3 ? (DMA_REGS->IFCR.ctcif3 = 1):
        channel_no == CHANNEL_4 ? (DMA_REGS->IFCR.ctcif4 = 1):
        channel_no == CHANNEL_5 ? (DMA_REGS->IFCR.ctcif5 = 1):
        channel_no == CHANNEL_6 ? (DMA_REGS->IFCR.ctcif6 = 1):
                                (DMA_REGS->IFCR.ctcif7 = 1);
    }

    inline void all_irq_clear()
    {
        channel_no == CHANNEL_1 ? (DMA_REGS->IFCR.cgif1 = 1):
        channel_no == CHANNEL_2 ? (DMA_REGS->IFCR.cgif2 = 1):
        channel_no == CHANNEL_3 ? (DMA_REGS->IFCR.cgif3 = 1):
        channel_no == CHANNEL_4 ? (DMA_REGS->IFCR.cgif4 = 1):
        channel_no == CHANNEL_5 ? (DMA_REGS->IFCR.cgif5 = 1):
        channel_no == CHANNEL_6 ? (DMA_REGS->IFCR.cgif6 = 1):
                                (DMA_REGS->IFCR.cgif7 = 1);
    }

    inline uint32_t current_counter() { return DMA_CHANNEL_REGS->CNDTR.ndt;}
};

#endif  // STM32L475_DMA_CPP_H
