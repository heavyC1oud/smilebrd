/**
 * @file stm32l475_rcc_reg.h
 *
 *       Reset and clock control registers
 *
 *       Based on RM0051
 *       Reference manual STM32L4xxx
 *           advanced ARM-based 32-bit MCUs
 *           April 2020 Rev 7 www.st.com
 */

#ifndef STM32L475_UART_REG_H
#define STM32L475_UART_REG_H

#include "regs.h"
#include "stm32l4xx.h"

/**
 * @brief Control register 1 (USART_CR1)
 *
 *        Address offset: 0x00
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_cr1_t,
    {
        mcu_reg_t ue             : 1; /*!< 0 UE: USART enable */
        mcu_reg_t uesm           : 1; /*!< 1 UESM: USART enable in Stop mode */
        mcu_reg_t re             : 1; /*!< 2 RE: Receiver enable */
        mcu_reg_t te             : 1; /*!< 3 TE: Transmitter enable */
        mcu_reg_t idleie         : 1; /*!< 4 IDLEIE: IDLE interrupt enable */
        mcu_reg_t rxneie         : 1; /*!< 5 RXNEIE: RXNE interrupt enable */
        mcu_reg_t tcie           : 1; /*!< 6 TCIE: Transmission complete interrupt enable */
        mcu_reg_t txeie          : 1; /*!< 7 TXEIE: TXE interrupt enable */
        mcu_reg_t peie           : 1; /*!< 8 PEIE: PE interrupt enable */
        mcu_reg_t ps             : 1; /*!< 9 PS: Parity selection */
        mcu_reg_t pce            : 1; /*!< 10 PCE: Parity control enable */
        mcu_reg_t wake           : 1; /*!< 11 WAKE: Wakeup method */
        mcu_reg_t m0             : 1; /*!< 12 M0: Word length */
        mcu_reg_t mme            : 1; /*!< 13 MME: Mute mode enable */
        mcu_reg_t cmie           : 1; /*!< 14 CMIE: Character match interrupt enable */
        mcu_reg_t over8          : 1; /*!< 15 OVER8: Oversampling mode */
        mcu_reg_t dedt           : 5; /*!< 20:16 DEDT[4:0] Driver Enable de-assertion time */
        mcu_reg_t deat           : 5; /*!< 25:21 DEAT[4:0] Driver Enable assertion time */
        mcu_reg_t rtoie          : 1; /*!< 26 RTOIE: Receiver timeout interrupt enable */
        mcu_reg_t eobie          : 1; /*!< 27 EOBIE: End of block interrupt enable */
        mcu_reg_t m1             : 1; /*!< 28 M1: Word length */
        mcu_reg_t Reserved29_31  : 3; /*!< 31:29 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Control register 2 (USART_CR2)
 *
 *        Address offset: 0x04
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_cr2_t,
    {
        mcu_reg_t Reserved0_3    : 4; /*!< 3:0 Reserved, must be kept at reset value */
        mcu_reg_t addm7          : 1; /*!< 4 ADDM7: 7-bit Address Detection/4-bit Address Detection */
        mcu_reg_t lbdl           : 1; /*!< 5 LBDL: lin break detection length */
        mcu_reg_t lbdie          : 1; /*!< 6 LBDIE: LIN break detection interrupt enable */
        mcu_reg_t Reserved7      : 1; /*!< 7 Reserved, must be kept at reset value */
        mcu_reg_t lbcl           : 1; /*!< 8 LBCL: Last bit clock pulse */
        mcu_reg_t cpha           : 1; /*!< 9 CPHA: Clock phase */
        mcu_reg_t cpol           : 1; /*!< 10 CPOL: Clock polarity */
        mcu_reg_t clken          : 1; /*!< 11 CLKEN: Clock enable */
        mcu_reg_t stop           : 2; /*!< 13:12 STOP[1:0]: STOP bits */
        mcu_reg_t linen          : 1; /*!< 14 LINEN: LIN mode enable */
        mcu_reg_t swap           : 1; /*!< 15 SWAP: Swap TX/RX pins */
        mcu_reg_t rxinv          : 1; /*!< 16 RXINV: RX pin active level inversion */
        mcu_reg_t txinv          : 1; /*!< 17 TXINV: TX pin active level inversion */
        mcu_reg_t datainv        : 1; /*!< 18 DATAINV: Binary data inversion */
        mcu_reg_t msbfirst       : 1; /*!< 19 MSBFIRST: Most significant bit first */
        mcu_reg_t abren          : 1; /*!< 20 ABREN: Auto baud rate enable */
        mcu_reg_t abrmod         : 2; /*!< 22:21 ABRMOD[1:0] Auto baud rate mode */
        mcu_reg_t rtoen          : 1; /*!< 23 RTOEN: Receiver timeout enable */
        mcu_reg_t add3_0         : 4; /*!< 27:24 ADD[3:0] Address of the USART node */
        mcu_reg_t add7_4         : 4; /*!< 31:28 ADD[7:4] Address of the USART node */
    }
);

/**
 * @brief Control register 3 (USART_CR3)
 *
 *        Address offset: 0x08
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_cr3_t,
    {
        mcu_reg_t eie            : 1; /*!< 0 EIE: Error interrupt enable */
        mcu_reg_t iren           : 1; /*!< 1 IREN: IrDA mode enable */
        mcu_reg_t irlp           : 1; /*!< 2 IRLP: IrDA low-power */
        mcu_reg_t hdsel          : 1; /*!< 3 HDSEL: Half-duplex selection */
        mcu_reg_t nack           : 1; /*!< 4 NACK: Smartcard NACK enable */
        mcu_reg_t scen           : 1; /*!< 5 SCEN: Smartcard mode enable */
        mcu_reg_t dmar           : 1; /*!< 6 DMAR: DMA enable receiver */
        mcu_reg_t dmat           : 1; /*!< 7 DMAT: DMA enable transmitter */
        mcu_reg_t rtse           : 1; /*!< 8 RTSE: RTS enable */
        mcu_reg_t ctse           : 1; /*!< 9 CTSE: CTS enable */
        mcu_reg_t ctsie          : 1; /*!< 10 CTSIE: CTS interrupt enable */
        mcu_reg_t onebit         : 1; /*!< 11 ONEBIT: One sample bit method enable */
        mcu_reg_t ovrdis         : 1; /*!< 12 OVRDIS: Overrun Disable */
        mcu_reg_t ddre           : 1; /*!< 13 DDRE: DMA Disable on Reception Error */
        mcu_reg_t dem            : 1; /*!< 14 DEM: Driver enable mode */
        mcu_reg_t dep            : 1; /*!< 15 DEP: Driver enable polarity selection */
        mcu_reg_t Reserved16     : 1; /*!< 16 Reserved, must be kept at reset value */
        mcu_reg_t scarcnt        : 3; /*!< 19:17 SCARCNT[2:0] Smartcard auto-retry count */
        mcu_reg_t wus            : 2; /*!< 21:20 WUS[1:0] Wakeup from Stop mode interrupt flag selection */
        mcu_reg_t wufie          : 1; /*!< 22 WUFIE: Wakeup from Stop mode interrupt enable */
        mcu_reg_t ucesm          : 1; /*!< 23 UCESM: USART Clock Enable in Stop mode */
        mcu_reg_t TCBGTIE        : 1; /*!< 24 TCBGTIE: Transmission complete before guard time interrupt enable */
        mcu_reg_t Reserved25_31  : 7; /*!< 31:25 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Baud rate register (USART_BRR)
 *
 *        This register can only be written when the USART is disabled (UE=0). It may be
 *        automatically updated by hardware in auto baud rate detection mode.
 *
 *        Address offset: 0x0C
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_brr_t,
    {
        mcu_reg_t brr            : 16; /*!< 15:0 BRR[15:0] */
        mcu_reg_t Reserved16_31  : 16; /*!< 31:16 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Guard time and prescaler register (USART_GTPR)
 *
 *        Address offset: 0x10
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_gtpr_t,
    {
        mcu_reg_t psc            : 8; /*!< 7:0 PSC[7:0] Prescaler value */
        mcu_reg_t gt             : 8; /*!< 15:8 GT[7:0] Guard time value */
        mcu_reg_t Reserved16_31  : 16;/*!< 31:16 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Receiver timeout register (USART_RTOR)
 *
 *        Address offset: 0x14
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_rtor_t,
    {
        mcu_reg_t rto            : 24; /*!< 23:0 RTO[23:0] Receiver timeout value */
        mcu_reg_t blen           : 8; /*!< 31:24 BLEN[7:0] Block Length */
    }
);

/**
 * @brief Request register (USART_RQR)
 *
 *        Address offset: 0x18
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_rqr_t,
    {
        mcu_reg_t abrrq          : 1; /*!< 0 ABRRQ: Auto baud rate request */
        mcu_reg_t sbkrq          : 1; /*!< 1 SBKRQ: Send break request */
        mcu_reg_t mmrq           : 1; /*!< 2 MMRQ: Mute mode request */
        mcu_reg_t rxfrq          : 1; /*!< 3 RXFRQ: Receive data flush request */
        mcu_reg_t txfrq          : 1; /*!< 4 TXFRQ: Transmit data flush request */
        mcu_reg_t Reserved5_31   : 27;/*!< 31:5 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief Interrupt and status register (USART_ISR)
 *
 *        Address offset: 0x1C
 *        Reset value: 0x0200 0000
 */
MCU_REG_TYPE (
    uart_isr_t,
    {
        mcu_reg_t pe             : 1; /*!< 0 PE: Parity error */
        mcu_reg_t fe             : 1; /*!< 1 FE: Framing error */
        mcu_reg_t nf             : 1; /*!< 2 NF: Noise detected flag */
        mcu_reg_t ore            : 1; /*!< 3 ORE: Overrun error */
        mcu_reg_t idle           : 1; /*!< 4 IDLE: Idle line detected */
        mcu_reg_t rxne           : 1; /*!< 5 RXNE: Read data register not empty */
        mcu_reg_t tc             : 1; /*!< 6 TC: Transmission complete */
        mcu_reg_t txe            : 1; /*!< 7 TXE: Transmit data register empty */
        mcu_reg_t lbd            : 1; /*!< 8 LBDF: LIN break detection flag */
        mcu_reg_t ctsi           : 1; /*!< 9 CTSIF: CTS interrupt flag */
        mcu_reg_t cts            : 1; /*!< 10 CTS: CTS flag */
        mcu_reg_t rto            : 1; /*!< 11 RTOF: Receiver timeout */
        mcu_reg_t eob            : 1; /*!< 12 EOBF: End of block flag */
        mcu_reg_t Reserved13     : 1; /*!< 13 Reserved, must be kept at reset value */
        mcu_reg_t abre           : 1; /*!< 14 ABRE: Auto baud rate error */
        mcu_reg_t abr            : 1; /*!< 15 ABRF: Auto baud rate flag */
        mcu_reg_t busy           : 1; /*!< 16 BUSY: Busy flag */
        mcu_reg_t cm             : 1; /*!< 17 CMF: Character match flag */
        mcu_reg_t sbk            : 1; /*!< 18 SBKF: Send break flag */
        mcu_reg_t rwu            : 1; /*!< 19 RWU: Receiver wakeup from Mute mode */
        mcu_reg_t wu             : 1; /*!< 20 WUF: Wakeup from Stop mode flag */
        mcu_reg_t teack          : 1; /*!< 21 TEACK: Transmit enable acknowledge flag */
        mcu_reg_t reack          : 1; /*!< 22 REACK: Receive enable acknowledge flag */
        mcu_reg_t Reserved23_24  : 2; /*!< 24:23 Reserved, must be kept at reset value. */
        mcu_reg_t tcbgt          : 1; /*!< 25 TCBGT: Transmission complete before guard time completion */
        mcu_reg_t Reserved26_31  : 6; /*!< 31:26 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief Interrupt flag clear register (USART_ICR)
 *
 *        Address offset: 0x20
 *        Reset value: 0x0200 0000
 */
MCU_REG_TYPE (
    uart_icr_t,
    {
        mcu_reg_t pe             : 1; /*!< 0 PECF: Parity error clear flag */
        mcu_reg_t fe             : 1; /*!< 1 FECF: Framing error clear flag */
        mcu_reg_t n              : 1; /*!< 2 NCF: Noise detected clear flag */
        mcu_reg_t ore            : 1; /*!< 3 ORECF: Overrun error clear flag */
        mcu_reg_t idle           : 1; /*!< 4 IDLECF: Idle line detected clear flag */
        mcu_reg_t Reserved5      : 1; /*!< 5 Reserved, must be kept at reset value. */
        mcu_reg_t tc             : 1; /*!< 6 TCCF: Transmission complete clear flag */
        mcu_reg_t tcbgt          : 1; /*!< 7 TCBGTCF: Transmission completed before guard time clear flag */
        mcu_reg_t lbd            : 1; /*!< 8 LBDCF: LIN break detection clear flag */
        mcu_reg_t cts            : 1; /*!< 9 CTSCF: CTS clear flag */
        mcu_reg_t Reserved10     : 1; /*!< 10 Reserved, must be kept at reset value. */
        mcu_reg_t rto            : 1; /*!< 11 RTOCF: Receiver timeout clear flag */
        mcu_reg_t eob            : 1; /*!< 12 EOBCF: End of block clear flag */
        mcu_reg_t Reserved13_16  : 4; /*!< 16:13 Reserved, must be kept at reset value clear flag */
        mcu_reg_t cm             : 1; /*!< 17 CMCF: Character match clear flag */
        mcu_reg_t Reserved18_19  : 2; /*!< 19:18 Reserved, must be kept at reset value clear flag */
        mcu_reg_t wu             : 1; /*!< 20 WUCF: Wakeup from Stop mode clear flag */
        mcu_reg_t Reserved21_31  : 11; /*!< 31:21 Reserved, must be kept at reset value. */
    }
);

/**
 * @brief Receive data register (USART_RDR)
 *
 *        Address offset: 0x24
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_rdr_t,
    {
        mcu_reg_t rdr            : 9; /*!< 8:0 RDR[8:0]: Receive data value */
        mcu_reg_t Reserved9_31   : 23;/*!< 31:9 Reserved, must be kept at reset value */
    }
);

/**
 * @brief Transmit data register (USART_TDR)
 *
 *        Address offset: 0x28
 *        Reset value: 0x0000 0000
 */
MCU_REG_TYPE (
    uart_tdr_t,
    {
        mcu_reg_t tdr            : 9; /*!< 8:0 TDR[8:0]: Transmit data value */
        mcu_reg_t Reserved9_31   : 23;/*!< 31:9 Reserved, must be kept at reset value */
    }
);



/**
 * @brief USART register map
 */
typedef __packed struct
{

    volatile uart_cr1_t  CR1;   /*!< 1 0x00 */
    volatile uart_cr2_t  CR2;   /*!< 2 0x04 */
    volatile uart_cr3_t  CR3;   /*!< 3 0x08 */
    volatile uart_brr_t  BRR;   /*!< 4 0x0C */
    volatile uart_gtpr_t GTPR;  /*!< 5 0x10 */
    volatile uart_rtor_t RTOR;  /*!< 6 0x14 */
    volatile uart_rqr_t  RQR;   /*!< 7 0x18 */
    volatile uart_isr_t  ISR;   /*!< 8 0x1C */
    volatile uart_icr_t  ICR;   /*!< 9 0x20 */
    volatile uart_rdr_t  RDR;   /*!< 10 0x24 */
    volatile uart_tdr_t  TDR;   /*!< 11 0x28 */

} uart_regs_t;

CHECK_REG_MAP(uart_regs_t, 11);

#define UART1_REGS_ADDR (uart_regs_t*)USART1_BASE
#define UART2_REGS_ADDR (uart_regs_t*)USART2_BASE
#define UART3_REGS_ADDR (uart_regs_t*)USART3_BASE
#define UART4_REGS_ADDR (uart_regs_t*)UART4_BASE
#define UART5_REGS_ADDR (uart_regs_t*)UART5_BASE

#endif  // STM32L475_UART_REG_H
