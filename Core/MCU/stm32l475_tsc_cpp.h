/**
 * @file stm32l475_tsc_cpp.h
 *
 *       Touch sensing controller
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_TSC_CPP_H
#define STM32L475_TSC_CPP_H

#include "basis.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_tsc_reg.h"


typedef enum
{
    TSC_GROUP_1,
    TSC_GROUP_2,
    TSC_GROUP_3,
    TSC_GROUP_4,
    TSC_GROUP_5,
    TSC_GROUP_6,
    TSC_GROUP_7,
    TSC_GROUP_8,
} tsc_group_no_t;

typedef enum
{
    TSC_IO_1,
    TSC_IO_2,
    TSC_IO_3,
    TSC_IO_4,
} tsc_io_no_t;

typedef enum
{
    TSC_PRESC_1 = 0,
    TSC_PRESC_2,
    TSC_PRESC_4,
    TSC_PRESC_8,
    TSC_PRESC_16,
    TSC_PRESC_32,
    TSC_PRESC_64,
    TSC_PRESC_128,
} tsc_presc_t;

typedef enum
{
    TSC_PULSE_X1 = 0,
    TSC_PULSE_X2,
    TSC_PULSE_X3,
    TSC_PULSE_X4,
    TSC_PULSE_X5,
    TSC_PULSE_X6,
    TSC_PULSE_X7,
    TSC_PULSE_X8,
    TSC_PULSE_X9,
    TSC_PULSE_X10,
    TSC_PULSE_X11,
    TSC_PULSE_X12,
    TSC_PULSE_X13,
    TSC_PULSE_X14,
    TSC_PULSE_X15,
    TSC_PULSE_X16,
} tsc_pulse_dur_t;

typedef enum
{
    TSC_MAX_COUNT_255,
    TSC_MAX_COUNT_511,
    TSC_MAX_COUNT_1023,
    TSC_MAX_COUNT_2047,
    TSC_MAX_COUNT_4095,
    TSC_MAX_COUNT_8191,
    TSC_MAX_COUNT_16383,
} tsc_max_count_t;

typedef enum
{
    TSC_ACQ_MODE_NORMAL,
    TSC_ACQ_MODE_SYNC,
} tsc_acq_mode_t;

typedef enum
{
    TSC_SYNC_POL_FALLING,
    TSC_SYNC_POL_RISING,
} tsc_sync_pol_t;

typedef enum
{
    TSC_DEF_PP_LOW,
    TSC_DEF_IN_FLOAT,
} tsc_def_pin_t;


class tsc_t
{
private:
    static struct base_tsc_regs
    {
        tsc_regs_t* operator-> ()
        {
            return TSC_REGS_ADDR;
        }
    } TSC_REGS;

    static inline uint32_t get_rcc_clock()
    {
        return rcc_t::get_clock(RCC_CLOCK_AHB);
    };

    static inline void rcc_on()
    {
        rcc_t::ahb1_periph_on(RCC_AHB1_PERIPH_TSC);
    };

    static inline void rcc_off()
    {
        rcc_t::ahb1_periph_off(RCC_AHB1_PERIPH_TSC);
    };

    static inline void rcc_reset_on()
    {
        rcc_t::ahb1_periph_reset_on(RCC_AHB1_PERIPH_TSC);
    };

    static inline void rcc_reset_off()
    {
        rcc_t::ahb1_periph_reset_off(RCC_AHB1_PERIPH_TSC);
    };

    static inline bool is_rcc_on()
    {
        return rcc_t::is_ahb1_periph_on(RCC_AHB1_PERIPH_TSC);
    }

public:
    static void reset()
    {
        tsc_t::rcc_reset_on();
        tsc_t::rcc_reset_off();
    };

    static void init()
    {
        if(tsc_t::is_rcc_on() == false) {
            tsc_t::rcc_on();
            tsc_t::reset();
        }

        if(tsc_t::is_on() == false) {
            tsc_t::on();
            tsc_t::set_acq_mode(TSC_ACQ_MODE_NORMAL);
            tsc_t::set_def_pin_mode(TSC_DEF_PP_LOW);
            tsc_t::set_mcv(TSC_MAX_COUNT_16383);

            // set 5 MHz clock (80 MHz / 16)
            tsc_t::set_pg_psc(TSC_PRESC_16);
            // set 2 us pulse low (10 pulse on 5 MHz = 2 us)
            tsc_t::set_pulse_low(TSC_PULSE_X10);
            // set 2 us pulse high (10 pulse on 5 MHz = 2 us)
            tsc_t::set_pulse_high(TSC_PULSE_X10);

            NVIC_SetPriority(TSC_IRQn, 0x00);
            NVIC_EnableIRQ(TSC_IRQn);
        }
    }

    static inline void on() {TSC_REGS->CR.tsce = 1;}
    static inline void off() {TSC_REGS->CR.tsce = 0;}
    static inline bool is_on() {return (bool)(TSC_REGS->CR.tsce == 1);}

    static inline void start() {TSC_REGS->CR.start = 1;}

    static inline void set_acq_mode(tsc_acq_mode_t mode) {TSC_REGS->CR.am = mode;}
    static inline void set_sync_pol(tsc_sync_pol_t pol) {TSC_REGS->CR.syncpol = pol;}

    static inline void set_def_pin_mode(tsc_def_pin_t mode) {TSC_REGS->CR.iodef = mode;}

    static inline void set_mcv(tsc_max_count_t data) {TSC_REGS->CR.mcv = data;}

    static inline uint32_t get_mcv() {return (0x3FFF >> (6 - TSC_REGS->CR.mcv));}

    static inline void set_pg_psc(tsc_presc_t data) {TSC_REGS->CR.pgpsc = data;}

    static inline void set_pulse_low(tsc_pulse_dur_t data) {TSC_REGS->CR.ctpl = data;}

    static inline void set_pulse_high(tsc_pulse_dur_t data) {TSC_REGS->CR.ctph = data;}

    static inline void en_eoa_irq() {TSC_REGS->IER.eoaie = 1;}
    static inline void en_mce_irq() {TSC_REGS->IER.mceie = 1;}

    static inline void dis_eoa_irq() {TSC_REGS->IER.eoaie = 0;}
    static inline void dis_mce_irq() {TSC_REGS->IER.mceie = 0;}

    static inline bool is_en_eoa_irq() {return (TSC_REGS->IER.eoaie == 1);}
    static inline bool is_en_mce_irq() {return (TSC_REGS->IER.mceie == 1);}

    static inline void clear_eoa_irq() {TSC_REGS->ICR.eoaic = 1;}
    static inline void clear_mce_irq() {TSC_REGS->ICR.mceic = 1;}

    static inline bool is_eoa_irq() {return (TSC_REGS->ISR.eoaf == 1);}
    static inline bool is_mce_irq() {return (TSC_REGS->ISR.mcef == 1);}

    static inline void en_hyst(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOHCR.d32 |= (1 << ((group << 2) + io));}
    static inline void dis_hyst(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOHCR.d32 &= ~(1 << ((group << 2) + io));}

    static inline void en_switch(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOASCR.d32 |= (1 << ((group << 2) + io));}
    static inline void dis_switch(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOASCR.d32 &= ~(1 << ((group << 2) + io));}

    static inline void en_sampling(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOSCR.d32 |= (1 << ((group << 2) + io));}
    static inline void dis_sampling(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOSCR.d32 &= ~(1 << ((group << 2) + io));}
    static inline void dis_all_sampling() {TSC_REGS->IOSCR.d32 = 0;}

    static inline void en_sensing(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOCCR.d32 |= (1 << ((group << 2) + io));}
    static inline void dis_sensing(tsc_group_no_t group, tsc_io_no_t io) {TSC_REGS->IOCCR.d32 &= ~(1 << ((group << 2) + io));}
    static inline void dis_all_sensing() {TSC_REGS->IOCCR.d32 = 0;}

    static inline void en_group_acq(tsc_group_no_t group) {TSC_REGS->IOGCSR.gxe |= (1 << group);}
    static inline void dis_group_acq(tsc_group_no_t group) {TSC_REGS->IOGCSR.gxe &= ~(1 << group);}

    static inline bool is_group_acq(tsc_group_no_t group) {return ((TSC_REGS->IOGCSR.gxs & (1 << group)) != 0);}

    static uint32_t get_counter(tsc_group_no_t group) {return (TSC_REGS->IOGxCR[group].cnt);}
};

#endif  // STM32L475_TSC_CPP_H
