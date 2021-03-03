/**
 * @file stm32l475_flash_cpp.h
 *
 *       Flash memory interface
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_FLASH_CPP_H
#define STM32L475_FLASH_CPP_H

#include "stm32l4xx.h"
#include "stm32l475_rcc_cpp.h"
#include "stm32l475_flash_reg.h"

#define FLASH_BANK_SIZE 256      // flash memory bank size in pages

#define FLASH_PDKEY1             ((uint32_t)0x04152637)
#define FLASH_PDKEY2             ((uint32_t)0xFAFBFCFD)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
#define FLASH_OPT_KEY1           ((uint32_t)0x08192A3B)
#define FLASH_OPT_KEY2           ((uint32_t)0x4C5D6E7F)

typedef enum
{
    FLASH_LATENCY_0 = 0,    //!< Zero wait state
    FLASH_LATENCY_1 = 1,    //!< One wait state
    FLASH_LATENCY_2 = 2,    //!< Two wait states
    FLASH_LATENCY_3 = 3,    //!< Three wait states
    FLASH_LATENCY_4 = 4,    //!< Four wait states
} flash_latency_t;

typedef enum
{
    FLASH_STATUS_BUSY = 1,
    FLASH_STATUS_ERROR_OPERR,       //!< Operation error
    FLASH_STATUS_ERROR_PROG,        //!< Programming error
    FLASH_STATUS_ERROR_WRP,         //!< Write protect error
    FLASH_STATUS_ERROR_PGA,         //!< Programming alignment error
    FLASH_STATUS_ERROR_SIZE,        //!< Size error
    FLASH_STATUS_ERROR_PGS,         //!< Programming sequence error
    FLASH_STATUS_ERROR_MIS,         //!< Fast programming data miss error
    FLASH_STATUS_ERROR_FAST,        //!< Fast programming error
    FLASH_STATUS_ERROR_RD,          //!< PCROP error
    FLASH_STATUS_ERROR_OPTV,        //!< Option validity error
    FLASH_STATUS_COMPLETE,
} flash_status_t;

typedef enum
{
    FLASH_BANK_1 = 0,
    FLASH_BANK_2 = 1,
} flash_bank_num_t;

class flash_t
{
private:
    static struct flash_base_reg
    {
        flash_regs_t* operator-> ()
        {
            return FLASH_REGS_ADDR;
        }
    } FLASH_REGS;

public:
    static inline void set_acr(uint32_t data) {FLASH_REGS->ACR.d32 = data;};

    static inline void latency(flash_latency_t latency) {FLASH_REGS->ACR.latency = latency;};
    static inline flash_latency_t get_latency() {return (flash_latency_t)FLASH_REGS->ACR.latency;};

    static inline void prefetch_en() {FLASH_REGS->ACR.prften = 1;}
    static inline void prefetch_dis() {FLASH_REGS->ACR.prften = 0;}

    static inline void icache_en() {FLASH_REGS->ACR.icen = 1;}
    static inline void icache_dis() {FLASH_REGS->ACR.icen = 0;}
    static inline void icache_reset() {FLASH_REGS->ACR.icrst = 1;}

    static inline void dcache_en() {FLASH_REGS->ACR.dcen = 1;}
    static inline void dcache_dis() {FLASH_REGS->ACR.dcen = 0;}
    static inline void dcache_reset() {FLASH_REGS->ACR.dcrst = 1;}

    static inline void set_bank(flash_bank_num_t bank) {FLASH_REGS->CR.bker = bank;};

private:
    static flash_status_t get_status()
    {
        if(FLASH_REGS->SR.bsy == 1) {
            return FLASH_STATUS_BUSY;
        }
        if(FLASH_REGS->SR.operr == 1) {
            return FLASH_STATUS_ERROR_OPERR;
        }
        if(FLASH_REGS->SR.progerr == 1) {
            return FLASH_STATUS_ERROR_PROG;
        }
        if(FLASH_REGS->SR.wrperr == 1) {
            return FLASH_STATUS_ERROR_WRP;
        }
        if(FLASH_REGS->SR.pgaerr == 1) {
            return FLASH_STATUS_ERROR_PGA;
        }
        if(FLASH_REGS->SR.sizerr == 1) {
            return FLASH_STATUS_ERROR_SIZE;
        }
        if(FLASH_REGS->SR.pgserr == 1) {
            return FLASH_STATUS_ERROR_PGS;
        }
        if(FLASH_REGS->SR.miserr == 1) {
            return FLASH_STATUS_ERROR_MIS;
        }
        if(FLASH_REGS->SR.fasterr == 1) {
            return FLASH_STATUS_ERROR_FAST;
        }
        if(FLASH_REGS->SR.rderr == 1) {
            return FLASH_STATUS_ERROR_RD;
        }
        if(FLASH_REGS->SR.optverr == 1) {
            return FLASH_STATUS_ERROR_OPTV;
        }

        return FLASH_STATUS_COMPLETE;
    }

    static flash_status_t wait_last_operation()
    {
        flash_status_t status = FLASH_STATUS_BUSY;
        while(status == FLASH_STATUS_BUSY) {
            status = flash_t::get_status();
        }
        return status;
    }

public:
    static void unlock()
    {
        if(FLASH_REGS->CR.lock != 0) {
            FLASH_REGS->KEY.fkey = FLASH_KEY1;
            FLASH_REGS->KEY.fkey = FLASH_KEY2;
        }
    }

    static inline void lock() {FLASH_REGS->CR.lock = 1;}

    static void option_unlock()
    {
        if(FLASH_REGS->CR.optlock != 0)
        {
            FLASH_REGS->OPTKEY.optkey = FLASH_OPT_KEY1;
            FLASH_REGS->OPTKEY.optkey = FLASH_OPT_KEY2;
        }
    }

    static inline void option_lock()
    {
        FLASH_REGS->CR.optlock = 1;
    }

    static inline void clear_flag_all() {FLASH_REGS->SR.d32 = 0xC3FB;}

    static flash_status_t erase_page(uint32_t page)
    {
        flash_status_t status = flash_t::wait_last_operation();
        if(status != FLASH_STATUS_BUSY)
        {
            flash_t::clear_flag_all();
            page < FLASH_BANK_SIZE ? flash_t::set_bank(FLASH_BANK_1) : flash_t::set_bank(FLASH_BANK_2);
            FLASH_REGS->CR.per = 1;
            FLASH_REGS->CR.pnb = page;
            FLASH_REGS->CR.strt = 1;

            status = flash_t::wait_last_operation();

            FLASH_REGS->CR.per = 0;
            FLASH_REGS->CR.pnb = 0;
        }
        return status;
    }

    static flash_status_t erase_all()
    {
        flash_status_t status = flash_t::wait_last_operation();
        if(status == FLASH_STATUS_COMPLETE)
        {
            flash_t::clear_flag_all();
            FLASH_REGS->CR.mer1 = 1;
            FLASH_REGS->CR.mer2 = 1;
            FLASH_REGS->CR.strt =1;

            status = flash_t::wait_last_operation();

            FLASH_REGS->CR.mer1 = 0;
            FLASH_REGS->CR.mer2 = 0;
        }
        return status;
    }

    static flash_status_t program_dword(uint32_t address, uint64_t data)
    {
        flash_status_t status = flash_t::wait_last_operation();
        if(status == FLASH_STATUS_COMPLETE)
        {
            flash_t::clear_flag_all();
            FLASH_REGS->CR.pg = 1;
            *(__IO uint64_t*)address = data;

            status = flash_t::wait_last_operation();

            FLASH_REGS->CR.pg = 0;
        }
        return status;
    }
};

#endif  // STM32L475_FLASH_CPP_H
