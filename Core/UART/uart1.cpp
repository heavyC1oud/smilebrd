
#include "basis.h"
#include "board.h"
#include "stm32l475_pin_cpp.h"
#include "stm32l475_uart_cpp.h"
#include "stm32l475_dma_cpp.h"
#include "uart1.h"


// Uart receive buffer
static uint8_t buf_uart1[UART_1_BUF_SIZE];

// UART low level routines
uart_ll_t<UART_1> uart1_ll;

// UART DMA
dma_t<DMA_1, CHANNEL_4> dma_uart1_tx;
dma_t<DMA_1, CHANNEL_5> dma_uart1_rx;

// UART instance
Uart1_t uart(buf_uart1);


void Uart1_t::init()
{
    if((this->buf == 0) || (UART_1_BUF_SIZE == 0)) {
        return;
    }

    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOA);
    pin_uart1_tx.Mode(ALT_OUTPUT);
    pin_uart1_tx.Alternate(ALT_FUNC7);
    pin_uart1_rx.Mode(ALT_INPUT);
    pin_uart1_rx.Alternate(ALT_FUNC7);

    // UART settings
    uart1_ll.init(115200, UART_PARITY_NONE, UART_DATABIT_8, UART_STOPBIT_1, UART_1_ISR_PRIORITY);

    // Tx
    // DMA Tx
    dma_uart1_tx.init(DMA1_CHANNEL4_2_USART1_TX, DMA_DIR_MEM2PERIPH, OFF, DMA_PRIOR_HIGH, DMA_DATASIZE_8, ON, DMA_DATASIZE_8, OFF);
    // set DMA peripheral address
    dma_uart1_tx.periph_addr(uart1_ll.get_tdr_addr());

    // enable DMA mode for UART transmitter
    uart1_ll.dma_tx_en();

    // DMA Rx
    dma_uart1_rx.init(DMA1_CHANNEL5_2_USART1_RX, DMA_DIR_PERIPH2MEM, OFF, DMA_PRIOR_HIGH, DMA_DATASIZE_8, ON, DMA_DATASIZE_8, OFF);
    // set DMA peripheral address
    dma_uart1_rx.periph_addr(uart1_ll.get_rdr_addr());
    // set DMA receive buffer address
    dma_uart1_rx.memory_addr(reinterpret_cast<uint32_t>(this->buf));
    // set DMA receive buffer size
    dma_uart1_rx.data_size(UART_1_BUF_SIZE);
    // enable DMA channel
    dma_uart1_rx.on();

    // config receiver timeout
    this->setRxDelay(20);
    uart1_ll.rto_en();

    // enable UART receiver
    uart1_ll.receive_en();
    // enable DMA mode for UART receiver
    uart1_ll.dma_rx_en();

    // enable UART
    uart1_ll.on();

    // enable RTO interrupt
    uart1_ll.rtoie_irq_en();
    // enable TC interrupt
    uart1_ll.tc_irq_en();
}


/**
 * @brief Wait for UART transmitting ends
 */
void Uart1_t::waitTxFree()
{
    while(dma_uart1_tx.is_on());
}


/**
 * @brief Is UART currently transmitting
 */
bool Uart1_t::isTxBusy()
{
    return dma_uart1_tx.is_on();
}


/**
 * @brief UART data transmit
 *
 * @param data: data to transmit
 * @param size: size of data
 */
void Uart1_t::sendData(uint8_t* data, uint16_t size)
{
    // disable receiver
    uart1_ll.receive_dis();
    // enable transmitter
    uart1_ll.transmit_en();

    dma_uart1_tx.data_size(size);
    dma_uart1_tx.memory_addr(reinterpret_cast<uint32_t>(data));
    dma_uart1_tx.on();
}


/**
 * @brief Set baudrate
 *
 * @param baudrate: baudrate
 */
void Uart1_t::setBaudrate(uint32_t baudrate)
{
    uart1_ll.set_baudrate(baudrate);
}


/**
 * @brief Get receive buffer index
 * @return: buffer index
 */
uint16_t Uart1_t::getRxBufIndex()
{
    return this->rxBufIndex;
}


/**
 * @brief Set receive buffer index
 * @param index: buffer index
 */
void Uart1_t::setRxBufIndex(uint16_t index)
{
    this->rxBufIndex = index;
}


/**
 * @brief Get transmit buffer index
 * @return buffer index
 */
uint16_t Uart1_t::getTxBufIndex()
{
    return this->txBufIndex;
}


/**
 * @brief Set transmit buffer index
 * @param index: buffer index
 */
void Uart1_t::setTxBufIndex(uint16_t index)
{
    this->txBufIndex = index;
}


/**
 * @brief Get pointer to receive buffer
 * @return: buffer
 */
uint8_t* Uart1_t::getBuf()
{
    return this->buf;
}


/**
 * @brief Enable receiver
 */
void Uart1_t::receiveEn()
{
    uart1_ll.receive_en();
}


/**
 * @brief Disable receiver
 */
void Uart1_t::receiveDis()
{
    uart1_ll.receive_dis();
    uart1_ll.rto_flag_clear();
}


/**
 * @brief Set RTOR delay in milliseconds
 *
 * @param delay: delay in milliseconds
 */
void Uart1_t::setRxDelay(uint16_t delay)
{
    uart1_ll.set_rtor((uart1_ll.get_baudrate() * delay) / 1000);
}


/**
 * @brief UART1 IRQ
 */
extern "C" void USART1_IRQHandler(void)
{
    // receive complete
    if(uart1_ll.is_rto_irq() == true) {
        // clear RTO flag
        uart1_ll.rto_flag_clear();
        // calculate received data
        uart.setRxBufIndex(UART_1_BUF_SIZE - dma_uart1_rx.current_counter());

        // disable DMA channel
        dma_uart1_rx.off();
        while(dma_uart1_rx.is_on() == true);
        // clear dma flags
        dma_uart1_rx.all_irq_clear();
        // set DMA receive buffer size
        dma_uart1_rx.data_size(UART_1_BUF_SIZE);
        // enable DMA channel
        dma_uart1_rx.on();
    }

    // transmit complete
    if(uart1_ll.is_tc_irq() == true) {
        // disable DMA channel
        dma_uart1_tx.off();
        // clear dma flags
        dma_uart1_tx.all_irq_clear();
        // clear uart TC flag
        uart1_ll.tc_flag_clear();
        // disable tranmitter
        uart1_ll.transmit_dis();
        // enable receiver
        uart1_ll.receive_en();
    }
}
