/**
 * @file stm32l475_i2c_cpp.h
 *
 *       Inter-integrated circuit interface
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_I2C_CPP_H
#define STM32L475_I2C_CPP_H

#include "basis.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_i2c_reg.h"

typedef enum
{
    I2C_1,
    I2C_2,
    I2C_3,
} i2c_num_t;

typedef enum
{
    I2C_MASTER_WRITE,
    I2C_MASTER_READ,
} i2c_dir_t;




template<i2c_num_t i2c_num>
class i2c_ll_t
{
private:
    static struct i2c_base_reg
    {
        i2c_regs_t* operator-> ()
        {
            return
                i2c_num == I2C_1 ? I2C1_REGS_ADDR:
                i2c_num == I2C_2 ? I2C2_REGS_ADDR:
                                   I2C3_REGS_ADDR;
        }
    } I2C_REGS;

    static struct i2c_base_irq
    {
        operator IRQn_Type()
        {
            return
                i2c_num == I2C_1 ? I2C1_EV_IRQn:
                i2c_num == I2C_2 ? I2C2_EV_IRQn:
                                   I2C3_EV_IRQn;
        }
    } I2Cx_IRQn;

    uint32_t get_rcc_clock()
    {
        return rcc_t::get_clock(RCC_CLOCK_APB1);
    }

    void rcc_on()
    {
        i2c_num == I2C_1 ? rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_I2C1):
        i2c_num == I2C_2 ? rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_I2C2):
                           rcc_t::apb1en1_periph_on(RCC_APB1_PERIPH_I2C3);
    }

    void rcc_off()
    {
        i2c_num == I2C_1 ? rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_I2C1):
        i2c_num == I2C_2 ? rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_I2C2):
                           rcc_t::apb1en1_periph_off(RCC_APB1_PERIPH_I2C3);
    }

    void rcc_reset_on()
    {
        i2c_num == I2C_1 ? rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_I2C1):
        i2c_num == I2C_2 ? rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_I2C2):
                           rcc_t::apb1rst1_periph_reset_on(RCC_APB1_PERIPH_I2C3);
    }

    void rcc_reset_off()
    {
        i2c_num == I2C_1 ? rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_I2C1):
        i2c_num == I2C_2 ? rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_I2C2):
                           rcc_t::apb1rst1_periph_reset_off(RCC_APB1_PERIPH_I2C3);
    }

    bool is_rcc_on()
    {
        return
            i2c_num == I2C_1 ? rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_I2C1):
            i2c_num == I2C_2 ? rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_I2C2):
                               rcc_t::is_apb1en1_periph_on(RCC_APB1_PERIPH_I2C3);
    }

public:
    i2c_ll_t()
    {
        if(this->is_rcc_on()) {
            this->reset();
            this->rcc_off();
        }
        NVIC_DisableIRQ(this->I2Cx_IRQn);
    }

    inline void reset()
    {
        this->rcc_reset_on();
        this->rcc_reset_off();
    }

    void init(uint8_t adr, uint8_t priority)
    {
        if (this->is_rcc_on()) {
            return;
        }
        this->rcc_on();
        this->reset();

        this->off();

        // timings for 80 MHz clock, 100 kHz i2c (TIMINGR = 0x1090 9CEC)
        this->set_timing_scll(0xEC);
        this->set_timing_sclh(0x9C);
        this->set_timing_sdadel(0x00);
        this->set_timing_scldel(0x09);
        this->set_timing_presc(0x01);

        this->on();

        this->set_oa1_7bit();
        this->set_oa1(adr);
        this->en_oa1();

        NVIC_SetPriority(this->I2Cx_IRQn, priority);
        NVIC_EnableIRQ(this->I2Cx_IRQn);
    }

    inline uint32_t get_isr() {return I2C_REGS->ISR.d32;}

    inline uint32_t get_rdr() {return I2C_REGS->RXDR.d32;}
    inline uint32_t get_tdr() {return I2C_REGS->TXDR.d32;}

    inline uint32_t get_rdr_addr() {return (uint32_t)&I2C_REGS->RXDR.d32;}
    inline uint32_t get_tdr_addr() {return (uint32_t)&I2C_REGS->TXDR.d32;}

    inline void on() {I2C_REGS->CR1.pe = 1;}
    inline void off() {I2C_REGS->CR1.pe = 0;}
    inline bool is_on() {return (I2C_REGS->CR1.pe == 1);}

    inline void dma_rx_en() {I2C_REGS->CR1.rxdmaen = 1;}
    inline void dma_rx_dis() {I2C_REGS->CR1.rxdmaen = 0;}

    inline void dma_tx_en() {I2C_REGS->CR1.txdmaen = 1;}
    inline void dma_tx_dis() {I2C_REGS->CR1.txdmaen = 0;}

    inline void en_stretch() {I2C_REGS->CR1.nostretch = 0;}
    inline void dis_stretch() {I2C_REGS->CR1.nostretch = 1;}

    inline void set_addr_master(uint8_t adr) {I2C_REGS->CR2.sadd = (adr << 1);}
    inline void set_dir_master(i2c_dir_t dir) {I2C_REGS->CR2.rd_wrn = dir;}
    inline void start() {I2C_REGS->CR2.start = 1;}
    inline void stop() {I2C_REGS->CR2.stop = 1;}
    inline void set_nack() {I2C_REGS->CR2.nack = 1;}
    inline void set_nbytes(uint8_t data) {I2C_REGS->CR2.nbytes = data;}

    inline void en_reload() {I2C_REGS->CR2.reload = 1;}
    inline void dis_reload() {I2C_REGS->CR2.reload = 0;}

    inline void en_autoend() {I2C_REGS->CR2.autoend = 1;}
    inline void dis_autoend() {I2C_REGS->CR2.autoend = 0;}

    inline void en_oa1() {I2C_REGS->OAR1.oa1en = 1;}
    inline void dis_oa1() {I2C_REGS->OAR1.oa1en = 0;}
    inline void set_oa1_7bit() {I2C_REGS->OAR1.oa1mode = 0;}
    inline void set_oa1_10bit() {I2C_REGS->OAR1.oa1mode = 1;}
    inline void set_oa1(uint8_t adr) {I2C_REGS->OAR1.oa1 = (adr << 1);}

    inline void en_oa2() {I2C_REGS->OAR2.oa1en = 1;}
    inline void dis_oa2() {I2C_REGS->OAR2.oa1en = 0;}
    inline void set_oa2_mask(uint8_t mask) {I2C_REGS->OAR2.oa2msk = mask;}
    inline void set_oa2(uint8_t adr) {I2C_REGS->OAR2.oa2 = adr;}

    inline void set_timing_scll(uint8_t data) {I2C_REGS->TIMING.scll = data;}
    inline void set_timing_sclh(uint8_t data) {I2C_REGS->TIMING.sclh = data;}
    inline void set_timing_sdadel(uint8_t data) {I2C_REGS->TIMING.sdadel = data;}
    inline void set_timing_scldel(uint8_t data) {I2C_REGS->TIMING.scldel = data;}
    inline void set_timing_presc(uint8_t data) {I2C_REGS->TIMING.presc = data;}
    inline void set_timing(uint32_t data) {I2C_REGS->TIMING.d32 = data;}
    inline uint32_t get_timing() {return I2C_REGS->TIMING.d32;}


    inline bool is_txe_flag() {return (I2C_REGS->ISR.txe == 1);}
    inline bool is_txis_flag() {return (I2C_REGS->ISR.txis == 1);}
    inline bool is_rxne_flag() {return (I2C_REGS->ISR.rxne == 1);}
    inline bool is_addr_match_flag() {return (I2C_REGS->ISR.addr == 1);}
    inline bool is_nack_flag() {return (I2C_REGS->ISR.nackf == 1);}
    inline bool is_stop_flag() {return (I2C_REGS->ISR.stopf == 1);}
    inline bool is_tc_flag() {return (I2C_REGS->ISR.tc == 1);}
    inline bool is_tcr_flag() {return (I2C_REGS->ISR.tcr == 1);}
    inline bool is_berr_flag() {return (I2C_REGS->ISR.berr == 1);}
    inline bool is_arlo_flag() {return (I2C_REGS->ISR.arlo == 1);}
    inline bool is_ovr_flag() {return (I2C_REGS->ISR.ovr == 1);}
    inline bool is_pecerr_flag() {return (I2C_REGS->ISR.pecerr == 1);}
    inline bool is_busy_flag() {return (I2C_REGS->ISR.busy == 1);}
    inline bool is_dir_flag() {return (I2C_REGS->ISR.dir == 1);}

    inline void addr_flag_clear() {I2C_REGS->ICR.addrcf = 1;}
    inline void nack_flag_clear() {I2C_REGS->ICR.nackcf = 1;}
    inline void stop_flag_clear() {I2C_REGS->ICR.stopcf = 1;}
    inline void berr_flag_clear() {I2C_REGS->ICR.berrcf = 1;}
    inline void arlo_flag_clear() {I2C_REGS->ICR.arlocf = 1;}
    inline void ovr_flag_clear() {I2C_REGS->ICR.ovrcf = 1;}
    inline void pec_flag_clear() {I2C_REGS->ICR.peccf = 1;}

    inline uint32_t get_pec() {return I2C_REGS->PECR.d32;}

    inline void en_tx_irq() {I2C_REGS->CR1.txie = 1;}
    inline void en_rx_irq() {I2C_REGS->CR1.rxie = 1;}
    inline void en_addr_irq() {I2C_REGS->CR1.addrie = 1;}
    inline void en_nack_irq() {I2C_REGS->CR1.nackie = 1;}
    inline void en_stop_irq() {I2C_REGS->CR1.stopie = 1;}
    inline void en_tc_irq() {I2C_REGS->CR1.tcie = 1;}
    inline void en_err_irq() {I2C_REGS->CR1.errie = 1;}

    inline void dis_tx_irq() {I2C_REGS->CR1.txie = 0;}
    inline void dis_rx_irq() {I2C_REGS->CR1.rxie = 0;}
    inline void dis_addr_irq() {I2C_REGS->CR1.addrie = 0;}
    inline void dis_nack_irq() {I2C_REGS->CR1.nackie = 0;}
    inline void dis_stop_irq() {I2C_REGS->CR1.stopie = 0;}
    inline void dis_tc_irq() {I2C_REGS->CR1.tcie = 0;}
    inline void dis_err_irq() {I2C_REGS->CR1.errie = 0;}

    inline bool is_tx_irq() {return ((I2C_REGS->CR1.txie == 1) && (I2C_REGS->ISR.txis == 1));}
    inline bool is_rx_irq() {return ((I2C_REGS->CR1.rxie == 1) && (I2C_REGS->ISR.rxne == 1));}
    inline bool is_addr_irq() {return ((I2C_REGS->CR1.addrie == 1) && (I2C_REGS->ISR.addr == 1));}
    inline bool is_nack_irq() {return ((I2C_REGS->CR1.nackie == 1) && (I2C_REGS->ISR.nackf == 1));}
    inline bool is_stop_irq() {return ((I2C_REGS->CR1.stopie == 1) && (I2C_REGS->ISR.stopf == 1));}
    inline bool is_tc_irq() {return ((I2C_REGS->CR1.tcie == 1) && (I2C_REGS->ISR.tc == 1));}
    inline bool is_err_irq() {return ((I2C_REGS->CR1.errie == 1) && (I2C_REGS->ISR.berr == 1));}

    inline char getch() {return (I2C_REGS->RXDR.rxdata);}

    /**
     * @brief Send one byte
     *
     * @param ch - data byte
     */
    void putch(char ch)
    {
        if(!this->is_on()) {
            return;
        }
        I2C_REGS->TXDR.txdata = ch;
    }

    /**
     * @brief Send string
     *
     * @param str - pointer to string data
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
     * @brief Send data array
     *
     * @param data - pointer to data array
     * @param size - size of data to send
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

#endif  // STM32L475_I2C_CPP_H
