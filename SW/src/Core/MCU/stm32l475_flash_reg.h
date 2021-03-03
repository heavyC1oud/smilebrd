/**
 * @file stm32l475_flash_reg.h
 *
 *       Flash memory interface registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_FLASH_REG_H
#define STM32L475_FLASH_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief Flash access control register (FLASH_ACR)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0600
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    flash_acr_t,
    {
        mcu_reg_t latency        : 3; /*!< 2:0 LATENCY[2:0] Latency */
        mcu_reg_t Reserved3_7    : 5; /*!< 7:3 Reserved, must be kept at reset value */
        mcu_reg_t prften         : 1; /*!< 8 PRFTEN: Prefetch enable */
        mcu_reg_t icen           : 1; /*!< 9 ICEN: Instruction cache enable */
        mcu_reg_t dcen           : 1; /*!< 10 DCEN: Data cache enable */
        mcu_reg_t icrst          : 1; /*!< 11 ICRST: Instruction cache reset */
        mcu_reg_t dcrst          : 1; /*!< 12 DCRST: Data cache reset */
        mcu_reg_t runpd          : 1; /*!< 13 RUN_PD: Flash Power-down mode during Run or Low-power run mode */
        mcu_reg_t sleeppd        : 1; /*!< 14 SLEEP_PD: Flash Power-down mode during Sleep or Low-power run mode */
        mcu_reg_t Reserved15_31  : 17;/*!< 31:15 Reserved, must be kept cleared. */
    }
);

/**
 * @brief Flash Power-down key register (FLASH_PDKEYR)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word access
 *
 *        The following values must be programmed consecutively to unlock the RUN_PD bit in FLASH_ACR
 *        a) KEY1 = 0x0415 2637
 *        b) KEY2 = 0xFAFB FCFD
 */
MCU_REG_TYPE (
    flash_pdkeyr_t,
    {
        mcu_reg_t pdkey           : 32; /*!< 31:0 PDKEYR: Power-down in Run mode Flash key*/
    };
);


/**
 * @brief Flash key register (FLASH_KEYR)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word access
 *
 *        The following values must be programmed consecutively to unlock the FLASH_CR register
 *        and allow programming/erasing it:
 *        a) KEY1 = 0x45670123
 *        b) KEY2 = 0xCDEF89AB
 */
MCU_REG_TYPE (
    flash_keyr_t,
    {
        mcu_reg_t fkey           : 32; /*!< 31:0 FKEYR: Flash key*/
    };
);

/**
 * @brief Flash option key register (FLASH_OPTKEYR)
 *
 *        Address offset: 0x0C
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word access
 *
 *        The following values must be programmed consecutively to unlock the FLASH_OPTCR
 *        register and allow programming it:
 *        a) OPTKEY1 = 0x08192A3B
 *        b) OPTKEY2 = 0x4C5D6E7F
 */
MCU_REG_TYPE (
    flash_opt_keyr_t,
    {
        mcu_reg_t optkey         : 32; /*!< 31:0 OPTKEYR: Option byte key */
    }
);

/**
 * @brief Flash status register (FLASH_SR)
 *
 *        Address offset: 0x10
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    flash_sr_t,
    {
        mcu_reg_t eop            : 1; /*!< 0 EOP: End of operation */
        mcu_reg_t operr          : 1; /*!< 1 OPERR: Operation error */
        mcu_reg_t Reserved2      : 1; /*!< 2 Reserved, must be kept at reset value */
        mcu_reg_t progerr        : 1; /*!< 3 PROGERR: Programming error */
        mcu_reg_t wrperr         : 1; /*!< 4 WRPERR: Write protection error */
        mcu_reg_t pgaerr         : 1; /*!< 5 PGAERR: Programming alignment error */
        mcu_reg_t sizerr         : 1; /*!< 6 SIZERR: Size error */
        mcu_reg_t pgserr         : 1; /*!< 7 PGSERR: Programming sequence error */
        mcu_reg_t miserr         : 1; /*!< 8 MISERR: Fast programming data miss error */
        mcu_reg_t fasterr        : 1; /*!< 9 FASTERR: Fast programming error */
        mcu_reg_t Reserved10_13  : 4; /*!< 13:10 Reserved, must be kept at reset value */
        mcu_reg_t rderr          : 1; /*!< 14 RDERR: PCROP read error */
        mcu_reg_t optverr        : 1; /*!< 15 OPTVERRERR: Option validity error */
        mcu_reg_t bsy            : 1; /*!< 16 BSY: Busy */
        mcu_reg_t Reserved17_31  : 15;/*!< 31:17 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Flash control register (FLASH_CR)
 *
 *        Address offset: 0x14
 *        Reset value: 0xC000 0000
 *        Access: no wait state when no Flash memory operation is ongoing, word, half-word and byte access.
 */
MCU_REG_TYPE (
    flash_cr_t,
    {
        mcu_reg_t pg             : 1; /*!< 0 PG: Programming */
        mcu_reg_t per            : 1; /*!< 1 PER: Page Erase */
        mcu_reg_t mer1           : 1; /*!< 2 MER1: Bank 1 Mass Erase */
        mcu_reg_t pnb            : 8; /*!< 10:3 PNB: Page number selection */
        mcu_reg_t bker           : 1; /*!< 11 BKER: Page number MSB (Bank selection) */
        mcu_reg_t Reserved12_14  : 3; /*!< 14:12 Reserved, must be kept at reset value */
        mcu_reg_t mer2           : 1; /*!< 15 MER2: Bank 2 Mass Erase */
        mcu_reg_t strt           : 1; /*!< 16 STRT: Start */
        mcu_reg_t optstrt        : 1; /*!< 17 OPTSTRT: Options modification start */
        mcu_reg_t fstpg          : 1; /*!< 18 FSTPG: Fast programming */
        mcu_reg_t Reserved19_23  : 5; /*!< 23:19 Reserved, must be kept at reset value */
        mcu_reg_t eopie          : 1; /*!< 24 EOPIE: End of operation interrupt enable */
        mcu_reg_t errie          : 1; /*!< 25 ERRIE: Error interrupt enable */
        mcu_reg_t rderrie        : 1; /*!< 26 RDERRIE: PCROP read error interrupt enbale */
        mcu_reg_t obllaunch      : 1; /*!< 27 OBL_LAUNCH: Force the option byte loading */
        mcu_reg_t Reserved28_29  : 2; /*!< 29:28 Reserved, must be kept at reset value */
        mcu_reg_t optlock        : 1; /*!< 30 OPTLOCK: Options lock */
        mcu_reg_t lock           : 1; /*!< 31 LOCK: FLASH_CR Lock */
    }
);

/**
 * @brief Flash ECC register (FLASH_ECCR)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0000
 *        Access: no wait state, word, half-word and byte access
 */
MCU_REG_TYPE (
    flash_eccr_t,
    {
        mcu_reg_t addrecc        : 19; /*!< 18:0 ADDR_ECC[18:0] ECC fail address */
        mcu_reg_t bkecc          : 1; /*!< 19 BK_ECC: ECC fail bank */
        mcu_reg_t sysfecc        : 1; /*!< 20 SYSF_ECC: System Flash ECC fail */
        mcu_reg_t Reserved21_23  : 3; /*!< 23:21 Reserved, must be kept at reset value */
        mcu_reg_t eccie          : 1; /*!< 24 ECCIE: ECC correction interrupt enable */
        mcu_reg_t Reserved25_29  : 5; /*!< 29:25 Reserved, must be kept at reset value */
        mcu_reg_t eccc           : 1; /*!< 30 ECCC: ECC correction */
        mcu_reg_t eccd           : 1; /*!< 31 ECCD: ECC detection */
    }
);

/**
 * @brief Flash option register (FLASH_OPTR)
 *
 *        Address offset: 0x20
 *        Reset value: 0xXXXX XXXX. Register bits 0 to 31 are loaded with values from Flash memory at OBL.
 *        Access: no wait state when no Flash memory operation is ongoing, word, half-word and byte access.
 */
MCU_REG_TYPE (
    flash_optr_t,
    {
        mcu_reg_t rdp            : 8; /*!< 7:0 RDP[7:0] Read protection level */
        mcu_reg_t borlev         : 3; /*!< 10:8 BOR_LEV[2:0] BOR reset level */
        mcu_reg_t Reserved11     : 1; /*!< 11 Reserved, must be kept at reset value */
        mcu_reg_t nrststop       : 1; /*!< 12 nRST_STOP: */
        mcu_reg_t nrststdby      : 1; /*!< 13 nRST_STDBY: */
        mcu_reg_t nrstshdw       : 1; /*!< 14 nRST_SHDW: */
        mcu_reg_t Reserved15     : 1; /*!< 15 Reserved, must be kept at reset value */
        mcu_reg_t idwgsw         : 1; /*!< 16 IDWG_SW: Independent watchdog selection */
        mcu_reg_t idwgstop       : 1; /*!< 17 IDWG_STOP: Independent watchdog counter freeze in Stop mode */
        mcu_reg_t idwgstdby      : 1; /*!< 18 IDWG_STDBY: Independent watchdog counter freeze in Standby mode */
        mcu_reg_t wwdgsw         : 1; /*!< 19 WWDG_SW: Window watchdog selection */
        mcu_reg_t bfb2           : 1; /*!< 20 BFB2: Dual-bank boot */
        mcu_reg_t dualbank       : 1; /*!< 21 DUALBANK: Dual-bank on 512 kb or 256 kb Flash memory devices */
        mcu_reg_t Reserved22     : 1; /*!< 22 Reserved, must be kept at reset value */
        mcu_reg_t nboot1         : 1; /*!< 23 nBOOT1: Boot configuration */
        mcu_reg_t sram2pe        : 1; /*!< 24 SRAM2_PE: SRAM2 parity check enable */
        mcu_reg_t sram2rst       : 1; /*!< 25 SRAM2_RST: SRAM2 Erase when system reset */
        mcu_reg_t nswboot0       : 1; /*!< 26 nSWBOOT0: Software BOOT0 */
        mcu_reg_t nboot0         : 1; /*!< 27 nBOOT0: nBOOT0 option bit */
        mcu_reg_t Reserved28_31  : 4; /*!< 31:28 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Flash interface register map
 */
typedef __packed struct
{
    volatile flash_acr_t      ACR;        /*!< 1 0x00 Flash access control register (FLASH_ACR) */
    volatile flash_pdkeyr_t   PDKEY;      /*!< 2 0x04 Flash Power-down key register (FLASH_PDKEYR) */
    volatile flash_keyr_t     KEY;        /*!< 3 0x08 Flash key register (FLASH_KEYR) */
    volatile flash_opt_keyr_t OPTKEY;     /*!< 4 0x0C Flash option key register (FLASH_OPTKEYR) */
    volatile flash_sr_t       SR;         /*!< 5 0x10 Flash status register (FLASH_SR) */
    volatile flash_cr_t       CR;         /*!< 6 0x14 Flash control register (FLASH_CR) */
    volatile flash_eccr_t     ECCR;       /*!< 7 0x18 Flash ECC register (FLASH_ECCR) */
    volatile mcu_reg_t        RESERVED0;  /*!< 8 0x1C */
    volatile flash_optr_t     OPTR;       /*!< 9 0x20 Flash option register (FLASH_OPTR) */
} flash_regs_t;

CHECK_REG_MAP(flash_regs_t, 9);

#define FLASH_REGS_ADDR (flash_regs_t*)FLASH_R_BASE

#endif  // STM32L475_FLASH_REG_H
