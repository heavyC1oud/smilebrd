/**
 * @file stm32l475_rcc_reg.h
 *
 *       Reset and clock control
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_UART_CPP_H
#define STM32L475_UART_CPP_H

#include "stm32l4xx.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_uart_reg.h"


typedef enum
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
    UART_5,
} uart_num_t;

typedef enum
{
    UART_PARITY_EVEN = 0,
    UART_PARITY_ODD  = 1,
    UART_PARITY_NONE = 2,
} uart_parity_t;

typedef enum
{
    UART_STOPBIT_1  = 0, /*! 1 Stop bit */
    UART_STOPBIT_05 = 1, /*! 0.5 Stop bit */
    UART_STOPBIT_2  = 2, /*! 2 Stop bits */
    UART_STOPBIT_15 = 3, /*! 1.5 Stop bit */
} uart_stopbit_t;

typedef enum
{
    UART_DATABIT_8  = 0, /*! 1 Start bit, 8 Data bits, n Stop bit */
    UART_DATABIT_9  = 1, /*! 1 Start bit, 9 Data bits, n Stop bit */
    UART_DATABIT_7  = 2, /*! 1 Start bit, 7 Data bits, n Stop bit */
} uart_databit_t;

typedef enum
{
    UART_RTS_ACTIVE_HIGH = 0, /*! RTS signal is active high */
    UART_RTS_ACTIVE_LOW = 1,  /*! RTS signal is active low */
} uart_rts_pol_t;

template<uart_num_t uart_num>
class uart_ll_t
{
private:
    static struct uart_base_reg
    {
        uart_regs_t* operator-> ()
        {
            return
                uart_num == UART_1 ? UART1_REGS_ADDR:
                uart_num == UART_2 ? UART2_REGS_ADDR:
                uart_num == UART_3 ? UART3_REGS_ADDR:
                uart_num == UART_4 ? UART4_REGS_ADDR:
                                     UART5_REGS_ADDR;
        }
    } UART_REGS;

    static struct uart_base_irq
    {
        operator IRQn_Type()
        {
            return
                uart_num == UART_1 ? USART1_IRQn:
                uart_num == UART_2 ? USART2_IRQn:
                uart_num == UART_3 ? USART3_IRQn:
                uart_num == UART_4 ? UART4_IRQn:
                                     UART5_IRQn;
        }
    } USARTx_IRQn;

    uint32_t get_rcc_clock()
    {
        return
            uart_num == UART_1 ? rcc_t::get_clock(RCC_CLOCK_APB2):
                                 rcc_t::get_clock(RCC_CLOCK_APB1);
    }

    void rcc_on()
    {
        uart_num == UART_1 ? rcc_t::apb2_periph_on(RCC_APB2_PERIPH_UART1):
        uart_num == UART_2 ? rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_UART2):
        uart_num == UART_3 ? rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_UART3):
        uart_num == UART_4 ? rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_UART4):
                             rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_UART5);
    }

    void rcc_off()
    {
        uart_num == UART_1 ? rcc_t::apb2_periph_off(RCC_APB2_PERIPH_UART1):
        uart_num == UART_2 ? rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_UART2):
        uart_num == UART_3 ? rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_UART3):
        uart_num == UART_4 ? rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_UART4):
                             rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_UART5);
    }

    void rcc_reset_on()
    {
        uart_num == UART_1 ? rcc_t::apb2_periph_reset_on(RCC_APB2_PERIPH_UART1):
        uart_num == UART_2 ? rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_UART2):
        uart_num == UART_3 ? rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_UART3):
        uart_num == UART_4 ? rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_UART4):
                             rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_UART5);
    }

    void rcc_reset_off()
    {
        uart_num == UART_1 ? rcc_t::apb2_periph_reset_off(RCC_APB2_PERIPH_UART1):
        uart_num == UART_2 ? rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_UART2):
        uart_num == UART_3 ? rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_UART3):
        uart_num == UART_4 ? rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_UART4):
                             rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_UART5);
    }

    bool is_rcc_on()
    {
        return
            uart_num == UART_1 ? rcc_t::is_apb2_periph_on(RCC_APB2_PERIPH_UART1):
            uart_num == UART_2 ? rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_UART2):
            uart_num == UART_3 ? rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_UART3):
            uart_num == UART_4 ? rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_UART4):
                                 rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_UART5);
    }

public:
    uart_ll_t()
    {
        if(this->is_rcc_on()) {
            this->reset();
            this->rcc_off();
        }
        NVIC_DisableIRQ(this->USARTx_IRQn);
    }

    inline void reset()
    {
        this->rcc_reset_on();
        this->rcc_reset_off();
    }

    void init(uint32_t baudrate, uart_parity_t parity, uart_databit_t data_len, uart_stopbit_t stopbit, uint8_t priority)
    {
        if (this->is_rcc_on()) {
            return;
        }
        this->rcc_on();
        this->reset();
        this->set_baudrate(baudrate);
        this->set_parity(parity);
        this->set_data_size(data_len);
        this->set_stop(stopbit);
        NVIC_SetPriority(this->USARTx_IRQn, priority);
        NVIC_EnableIRQ(this->USARTx_IRQn);
    }

    inline uint32_t get_isr() {return UART_REGS->ISR.d32;}

    inline uint32_t get_rdr() {return UART_REGS->RDR.d32;}
    inline uint32_t get_tdr() {return UART_REGS->TDR.d32;}

    inline uint32_t get_rdr_addr() {return (uint32_t)&UART_REGS->RDR.d32;}
    inline uint32_t get_tdr_addr() {return (uint32_t)&UART_REGS->TDR.d32;}

    inline void set_parity(uart_parity_t parity)
    {
        if(parity == UART_PARITY_NONE)
        {
            UART_REGS->CR1.pce = 0;
            return;
        }
        UART_REGS->CR1.pce = 1;
        UART_REGS->CR1.ps = parity;
    }

    inline void set_stop(uart_stopbit_t stop) {UART_REGS->CR2.stop = stop;}

    inline void set_baudrate(uint32_t baudrate)
    {
        UART_REGS->BRR.d32 = ((this->get_rcc_clock()/baudrate) << UART_REGS->CR1.over8);
    }

    inline uint32_t get_baudrate()
    {
        uint32_t over8 = UART_REGS->CR1.over8;
        return (this->get_rcc_clock()/(UART_REGS->BRR.d32 >> over8));
    }

    inline void set_data_size(uart_databit_t size)
    {
        UART_REGS->CR1.m0 = size & 0x1UL;
        UART_REGS->CR1.m1 = (size >> 1) & 0x1UL;
    }

    inline void on() {UART_REGS->CR1.ue = 1;}
    inline void off() {UART_REGS->CR1.ue = 0;}
    inline bool is_on() {return (UART_REGS->CR1.ue == 1);}

    inline void transmit_en () {UART_REGS->CR1.te = 1;}
    inline void transmit_dis () {UART_REGS->CR1.te = 0;}

    inline void receive_en () {UART_REGS->CR1.re = 1;}
    inline void receive_dis () {UART_REGS->CR1.re = 0;}

    inline void dma_rx_en () {UART_REGS->CR3.dmar = 1;}
    inline void dma_rx_dis () {UART_REGS->CR3.dmar = 0;}

    inline void dma_tx_en () {UART_REGS->CR3.dmat = 1;}
    inline void dma_tx_dis () {UART_REGS->CR3.dmat = 0;}

    inline void set_rtor(uint32_t data) {UART_REGS->RTOR.rto = data;}
    inline uint32_t get_rtor() {return UART_REGS->RTOR.rto;}

    inline void rto_en() {UART_REGS->CR2.rtoen = 1;}
    inline void rto_dis() {UART_REGS->CR2.rtoen = 0;}

    inline void driver_en() {UART_REGS->CR3.dem = 1;}
    inline void driver_dis() {UART_REGS->CR3.dem = 0;}

    inline void set_driver(uart_rts_pol_t data) {UART_REGS->CR3.dep = data;}

    inline bool is_pe_flag() {return (UART_REGS->ISR.pe == 1);}
    inline bool is_fe_flag() {return (UART_REGS->ISR.fe == 1);}
    inline bool is_nf_flag() {return (UART_REGS->ISR.nf == 1);}
    inline bool is_ore_flag() {return (UART_REGS->ISR.ore == 1);}
    inline bool is_idle_flag() {return (UART_REGS->ISR.idle == 1);}
    inline bool is_rxne_flag() {return (UART_REGS->ISR.rxne == 1);}
    inline bool is_tc_flag() {return (UART_REGS->ISR.tc == 1);}
    inline bool is_txe_flag() {return (UART_REGS->ISR.txe == 1);}
    inline bool is_lbd_flag() {return (UART_REGS->ISR.lbd == 1);}
    inline bool is_ctsi_flag() {return (UART_REGS->ISR.ctsi == 1);}
    inline bool is_cts_flag() {return (UART_REGS->ISR.cts == 1);}
    inline bool is_rto_flag() {return (UART_REGS->ISR.rto == 1);}
    inline bool is_eob_flag() {return (UART_REGS->ISR.eob == 1);}
    inline bool is_abr_flag() {return (UART_REGS->ISR.abr == 1);}
    inline bool is_busy_flag() {return (UART_REGS->ISR.busy == 1);}
    inline bool is_cm_flag() {return (UART_REGS->ISR.cm == 1);}
    inline bool is_sbk_flag() {return (UART_REGS->ISR.sbk == 1);}
    inline bool is_wuf_flag() {return (UART_REGS->ISR.wu == 1);}
    inline bool is_teack_flag() {return (UART_REGS->ISR.teack == 1);}
    inline bool is_reack_flag() {return (UART_REGS->ISR.reack == 1);}

    inline bool pe_flag_clear() {return (UART_REGS->ICR.pe = 1);}
    inline bool fe_flag_clear() {return (UART_REGS->ICR.fe = 1);}
    inline bool n_flag_clear() {return (UART_REGS->ICR.n = 1);}
    inline bool ore_flag_clear() {return (UART_REGS->ICR.ore = 1);}
    inline bool idle_flag_clear() {return (UART_REGS->ICR.idle = 1);}
    inline bool tc_flag_clear() {return (UART_REGS->ICR.tc = 1);}
    inline bool tcbgt_flag_clear() {return (UART_REGS->ICR.tcbgt = 1);}
    inline bool lbd_flag_clear() {return (UART_REGS->ICR.lbd = 1);}
    inline bool cts_flag_clear() {return (UART_REGS->ICR.cts = 1);}
    inline bool rto_flag_clear() {return (UART_REGS->ICR.rto = 1);}
    inline bool eob_flag_clear() {return (UART_REGS->ICR.eob = 1);}
    inline bool cm_flag_clear() {return (UART_REGS->ICR.cm = 1);}
    inline bool wu_flag_clear() {return (UART_REGS->ICR.wu = 1);}

    inline void idle_irq_en() {UART_REGS->CR1.idleie = 1;}
    inline void rxne_irq_en() {UART_REGS->CR1.rxneie = 1;}
    inline void tc_irq_en() {UART_REGS->CR1.tcie = 1;}
    inline void txe_irq_en() {UART_REGS->CR1.txeie = 1;}
    inline void pe_irq_en() {UART_REGS->CR1.peie = 1;}
    inline void cmie_irq_en() {UART_REGS->CR1.cmie = 1;}
    inline void rtoie_irq_en() {UART_REGS->CR1.rtoie = 1;}
    inline void eobie_irq_en() {UART_REGS->CR1.eobie = 1;}

    inline void idle_irq_dis() {UART_REGS->CR1.idleie = 0;}
    inline void rxne_irq_dis() {UART_REGS->CR1.rxneie = 0;}
    inline void tc_irq_dis() {UART_REGS->CR1.tcie = 0;}
    inline void txe_irq_dis() {UART_REGS->CR1.txeie = 0;}
    inline void pe_irq_dis() {UART_REGS->CR1.peie = 0;}
    inline void cmie_irq_dis() {UART_REGS->CR1.cmie = 0;}
    inline void rtoie_irq_dis() {UART_REGS->CR1.rtoie = 0;}
    inline void eobie_irq_dis() {UART_REGS->CR1.eobie = 0;}

    inline bool is_fe_irq() {return ((UART_REGS->CR3.eie == 1) && (UART_REGS->ISR.fe == 1));}
    inline bool is_pe_irq() {return ((UART_REGS->CR1.peie == 1) && (UART_REGS->ISR.pe == 1));}
    inline bool is_cts_irq() {return ((UART_REGS->CR3.ctsie == 1) && (UART_REGS->ISR.cts == 1));}
    inline bool is_lbdf_irq() {return ((UART_REGS->CR2.lbdie == 1) && (UART_REGS->ISR.lbd == 1));}
    inline bool is_txe_irq() {return ((UART_REGS->CR1.txeie == 1) && (UART_REGS->ISR.txe == 1));}
    inline bool is_tc_irq() {return ((UART_REGS->CR1.tcie == 1) &&( UART_REGS->ISR.tc == 1));}
    inline bool is_rxne_irq() {return ((UART_REGS->CR1.rxneie == 1) && (UART_REGS->ISR.rxne == 1));}
    inline bool is_idle_irq() {return ((UART_REGS->CR1.idleie == 1) && (UART_REGS->ISR.idle == 1));}
    inline bool is_rto_irq() {return ((UART_REGS->CR1.rtoie == 1) && (UART_REGS->ISR.rto == 1));}
    inline bool is_eob_irq() {return ((UART_REGS->CR1.eobie == 1) && (UART_REGS->ISR.eob == 1));}
    inline bool is_ore_rx_irq() {return ((UART_REGS->CR1.rxneie == 1) && (UART_REGS->ISR.ore == 1));}
    inline bool is_ore_er_irq() {return ((UART_REGS->CR3.eie == 1) && (UART_REGS->ISR.ore == 1));}
    inline bool is_nf_irq() {return ((UART_REGS->CR3.eie == 1) && (UART_REGS->ISR.nf == 1));}

    inline char getch() {return ( UART_REGS->RDR.dr );}

    /**
     * @brief Отправка одного байта без использования DMA.
     *
     * @param ch - передаваемый байт
     */
    void putch(char ch)
    {
        if(!this->is_on()) {
            return;
        }
        UART_REGS->TDR.dr = ch;
        while(this->is_tc_flag() == false);
    }

    /**
     * @brief Передача null-терминированной строки без использования DMA.
     *
     * @param str - указатель на строку оканчивающуюся 0.
     */
    void puts(char* str)
    {
        if(!this->is_on()) {
            return;
        }
        char* tmp_data = (char *)str;
        while(*tmp_data != 0)
        {
            this->putch(*tmp_data);
            tmp_data++;
        }
    }

    /**
     * @brief Передача массива данных без использования DMA.
     *
     * @param data - указатель на массив данных
     * @param size - количество передаваемых байт
     */
    void putdata(uint8_t* data, uint16_t size)
    {
        if(!this->is_on()) {
            return;
        }
        char* tmp_data = (char *)data;
        while (size != 0)
        {
            this->putch(*tmp_data);
            tmp_data++;
            size--;
        }
    }
};

#endif  // STM32L475_UART_CPP_H
