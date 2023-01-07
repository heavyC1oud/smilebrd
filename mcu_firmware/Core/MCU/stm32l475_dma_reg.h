/**
 * @file stm32l475_dma_reg.h
 *
 *       Direct memory access controller registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_DMA_REG_H
#define STM32L475_DMA_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief DMA interrupt status register (DMA_ISR)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_isr_t,
    {
        mcu_reg_t gif1           : 1; /*!< 0 GIF1: Global interrupt flag for channel 1 */
        mcu_reg_t tcif1          : 1; /*!< 1 TCIF1:Transfer complete flag for channel 1 */
        mcu_reg_t htif1          : 1; /*!< 2 HTIF1: Half transfer flag for channel 1 */
        mcu_reg_t teif1          : 1; /*!< 3 TEIF1:Transfer error flag for channel 1 */
        mcu_reg_t gif2           : 1; /*!< 4 GIF2: Global interrupt flag for channel 2 */
        mcu_reg_t tcif2          : 1; /*!< 5 TCIF2:Transfer complete flag for channel 2 */
        mcu_reg_t htif2          : 1; /*!< 6 HTIF2: Half transfer flag for channel 2 */
        mcu_reg_t teif2          : 1; /*!< 7 TEIF2:Transfer error flag for channel 2 */
        mcu_reg_t gif3           : 1; /*!< 8 GIF3: Global interrupt flag for channel 3 */
        mcu_reg_t tcif3          : 1; /*!< 9 TCIF3:Transfer complete flag for channel 3 */
        mcu_reg_t htif3          : 1; /*!< 10 HTIF3: Half transfer flag for channel 3 */
        mcu_reg_t teif3          : 1; /*!< 11 TEIF3:Transfer error flag for channel 3 */
        mcu_reg_t gif4           : 1; /*!< 12 GIF4: Global interrupt flag for channel 4 */
        mcu_reg_t tcif4          : 1; /*!< 13 TCIF4:Transfer complete flag for channel 4 */
        mcu_reg_t htif4          : 1; /*!< 14 HTIF4: Half transfer flag for channel 4 */
        mcu_reg_t teif4          : 1; /*!< 15 TEIF4:Transfer error flag for channel 4 */
        mcu_reg_t gif5           : 1; /*!< 16 GIF5: Global interrupt flag for channel 5 */
        mcu_reg_t tcif5          : 1; /*!< 17 TCIF5:Transfer complete flag for channel 5 */
        mcu_reg_t htif5          : 1; /*!< 18 HTIF5: Half transfer flag for channel 5 */
        mcu_reg_t teif5          : 1; /*!< 19 TEIF5:Transfer error flag for channel 5 */
        mcu_reg_t gif6           : 1; /*!< 20 GIF6: Global interrupt flag for channel 6 */
        mcu_reg_t tcif6          : 1; /*!< 21 TCIF6:Transfer complete flag for channel 6 */
        mcu_reg_t htif6          : 1; /*!< 22 HTIF6: Half transfer flag for channel 6 */
        mcu_reg_t teif6          : 1; /*!< 23 TEIF6:Transfer error flag for channel 6 */
        mcu_reg_t gif7           : 1; /*!< 24 GIF7: Global interrupt flag for channel 7 */
        mcu_reg_t tcif7          : 1; /*!< 25 TCIF7:Transfer complete flag for channel 7 */
        mcu_reg_t htif7          : 1; /*!< 26 HTIF7: Half transfer flag for channel 7 */
        mcu_reg_t teif7          : 1; /*!< 27 TEIF7:Transfer error flag for channel 7 */
        mcu_reg_t Reserved28_31  : 4; /*!< 31:28 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief DMA interrupt flag clear register (DMA_IFCR)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_ifcr_t,
    {
        mcu_reg_t cgif1          : 1; /*!< 0 CGIF1: Global interrupt flag clear for channel 1 */
        mcu_reg_t ctcif1         : 1; /*!< 1 CTCIF1:Transfer complete flag clear for channel 1 */
        mcu_reg_t chtif1         : 1; /*!< 2 CHTIF1: Half transfer flag clear for channel 1 */
        mcu_reg_t cteif1         : 1; /*!< 3 CTEIF1:Transfer error flag clear for channel 1 */
        mcu_reg_t cgif2          : 1; /*!< 4 CGIF2: Global interrupt flag clear for channel 2 */
        mcu_reg_t ctcif2         : 1; /*!< 5 CTCIF2:Transfer complete flag clear for channel 2 */
        mcu_reg_t chtif2         : 1; /*!< 6 CHTIF2: Half transfer flag clear for channel 2 */
        mcu_reg_t cteif2         : 1; /*!< 7 CTEIF2:Transfer error flag clear for channel 2 */
        mcu_reg_t cgif3          : 1; /*!< 8 CGIF3: Global interrupt flag clear for channel 3 */
        mcu_reg_t ctcif3         : 1; /*!< 9 CTCIF3:Transfer complete flag clear for channel 3 */
        mcu_reg_t chtif3         : 1; /*!< 10 CHTIF3: Half transfer flag clear for channel 3 */
        mcu_reg_t cteif3         : 1; /*!< 11 CTEIF3:Transfer error flag clear for channel 3 */
        mcu_reg_t cgif4          : 1; /*!< 12 CGIF4: Global interrupt flag clear for channel 4 */
        mcu_reg_t ctcif4         : 1; /*!< 13 CTCIF4:Transfer complete flag clear for channel 4 */
        mcu_reg_t chtif4         : 1; /*!< 14 CHTIF4: Half transfer flag clear for channel 4 */
        mcu_reg_t cteif4         : 1; /*!< 15 CTEIF4:Transfer error flag clear for channel 4 */
        mcu_reg_t cgif5          : 1; /*!< 16 CGIF5: Global interrupt flag clear for channel 5 */
        mcu_reg_t ctcif5         : 1; /*!< 17 CTCIF5:Transfer complete flag clear for channel 5 */
        mcu_reg_t chtif5         : 1; /*!< 18 CHTIF5: Half transfer flag clear for channel 5 */
        mcu_reg_t cteif5         : 1; /*!< 19 CTEIF5:Transfer error flag clear for channel 5 */
        mcu_reg_t cgif6          : 1; /*!< 20 CGIF6: Global interrupt flag clear for channel 6 */
        mcu_reg_t ctcif6         : 1; /*!< 21 CTCIF6:Transfer complete flag clear for channel 6 */
        mcu_reg_t chtif6         : 1; /*!< 22 CHTIF6: Half transfer flag clear for channel 6 */
        mcu_reg_t cteif6         : 1; /*!< 23 CTEIF6:Transfer error flag clear for channel 6 */
        mcu_reg_t cgif7          : 1; /*!< 24 CGIF7: Global interrupt flag clear for channel 7 */
        mcu_reg_t ctcif7         : 1; /*!< 25 CTCIF7:Transfer complete flag clear for channel 7 */
        mcu_reg_t chtif7         : 1; /*!< 26 CHTIF7: Half transfer flag clear for channel 7 */
        mcu_reg_t cteif7         : 1; /*!< 27 CTEIF7:Transfer error flag clear for channel 7 */
        mcu_reg_t Reserved28_31  : 4; /*!< 31:28 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief DMA channel x configuration register (DMA_CCRx)
 *
 *        This register is used to configure the concerned stream.
 *
 *        Address offset: 0x08 + 0x14 × (x - 1), (x = 1..7)
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_ccr_t,
    {
        mcu_reg_t en             : 1; /*!< 0 EN: Channel enable */
        mcu_reg_t tcie           : 1; /*!< 1 TCIE: Transfer complete interrupt enable */
        mcu_reg_t htie           : 1; /*!< 2 HTIE: Half transfer interrupt enable */
        mcu_reg_t teie           : 1; /*!< 3 TEIE: Transfer error interrupt enable */
        mcu_reg_t dir            : 1; /*!< 4 DIR: Data transfer direction */
        mcu_reg_t circ           : 1; /*!< 5 CIRC: Circular mode */
        mcu_reg_t pinc           : 1; /*!< 6 PINC: Peripheral increment mode */
        mcu_reg_t minc           : 1; /*!< 7 MINC: Memory increment mode */
        mcu_reg_t psize          : 2; /*!< 9:8 PSIZE[1:0] Peripheral size */
        mcu_reg_t msize          : 2; /*!< 11:10 MSIZE[1:0] Memory size */
        mcu_reg_t pl             : 2; /*!< 13:12 PL[1:0] Priority level */
        mcu_reg_t mem2mem        : 1; /*!< 14 MEM2MEM: Memory-to-memory mode */
        mcu_reg_t Reserved15_31  : 17; /*!< 31:15 Reserved, must be kept at reset value.*/
    }
);

/**
 * @brief DMA channel x number of data to transfer register (DMA_CNDTRx)
 *
 *        Address offset: 0x0C + 0x14 × (x - 1) (x = 1..7)
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_cndtr_t,
    {
        mcu_reg_t ndt            : 16;/*!< 15:0 NDT[15:0]: Number of data to transfer */
        mcu_reg_t Reserved16_31  : 16;/*!< 31:16 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief DMA channel x peripheral address register (DMA_CPARx)
 *
 *        Address offset: 0x10 + 0x14 × (x - 1) (x = 1..7)
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_cpar_t,
    {
        mcu_reg_t pa            : 32;/*!< 31:0 PA[31:0]: Peripheral address */
    }
);

/**
 * @brief DMA channel x memory address register (DMA_CMARx)
 *
 *        Address offset: 0x14 + 0x14 × (x - 1) (x = 1..7)
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_cmar_t,
    {
        mcu_reg_t ma           : 32;/*!< 31:0 MA[31:0]: Memory address */
    }
);


/**
 * @brief DMA channel selection register (DMA_CSELR)
 *
 *        Address offset: 0xA8
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    dma_cselr_t,
    {
        mcu_reg_t c1s            : 4; /*!< 3:0 C1S[3:0] DMA channel 1 selection */
        mcu_reg_t c2s            : 4; /*!< 3:0 C2S[3:0] DMA channel 2 selection */
        mcu_reg_t c3s            : 4; /*!< 3:0 C3S[3:0] DMA channel 3 selection */
        mcu_reg_t c4s            : 4; /*!< 3:0 C4S[3:0] DMA channel 4 selection */
        mcu_reg_t c5s            : 4; /*!< 3:0 C5S[3:0] DMA channel 5 selection */
        mcu_reg_t c6s            : 4; /*!< 3:0 C6S[3:0] DMA channel 6 selection */
        mcu_reg_t c7s            : 4; /*!< 3:0 C7S[3:0] DMA channel 7 selection */
        mcu_reg_t Reserved28_31  : 4; /*!< 31:28 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief DMA channel register map
 */
typedef __packed struct
{
    volatile dma_ccr_t    CCR;       /*!< 1 0x00 */
    volatile dma_cndtr_t  CNDTR;     /*!< 2 0x04 */
    volatile dma_cpar_t   CPAR;      /*!< 3 0x08 */
    volatile dma_cmar_t   CMAR;      /*!< 4 0x0C */
    volatile mcu_reg_t    RESERVED0; /*!< 5 0x10 */
} dma_regs_channel_t;

CHECK_REG_MAP(dma_regs_channel_t, 5);

/**
 * @brief DMA register map
 */
typedef __packed struct
{
    volatile dma_isr_t      ISR;            /*!< 1 0x00 */
    volatile dma_ifcr_t     IFCR;           /*!< 2 0x04 */
    dma_regs_channel_t      CHANNELS[7];    /*!< 3..37 0x08..0x90 */
    volatile mcu_reg_t      RESERVED0;      /*!< 38 0x94 */
    volatile mcu_reg_t      RESERVED1;      /*!< 39 0x98 */
    volatile mcu_reg_t      RESERVED2;      /*!< 40 0x9C */
    volatile mcu_reg_t      RESERVED3;      /*!< 41 0xA0 */
    volatile mcu_reg_t      RESERVED4;      /*!< 42 0xA4 */
    volatile dma_cselr_t    CSELR;          /*!< 43 0xA8 */
} dma_regs_t;

CHECK_REG_MAP(dma_regs_t, (8 + 5 * 7));

#define DMA1_REGS_ADDR (dma_regs_t*)DMA1_BASE
#define DMA2_REGS_ADDR (dma_regs_t*)DMA2_BASE

#endif  // STM32L475_DMA_REG_H
