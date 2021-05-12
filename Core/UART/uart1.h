#ifndef _UART_1_H_
#define _UART_1_H_

typedef enum {
    UART_1_ISR_PRIORITY = 1,
    UART_1_BUF_SIZE = 512,
} UART_1_param_t;

typedef enum {
    UART_BAUD_1200,
    UART_BAUD_2400,
    UART_BAUD_4800,
    UART_BAUD_9600,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200,
} UART_baud_t;

class Uart1_t
{
private:
    uint16_t rxBufIndex;      // current byte index in RX buffer
    uint16_t txBufIndex;      // current byte index in TX buffer
    uint8_t* buf;             // pointer to RX/TX buffer

public:
    Uart1_t(uint8_t* buf):
        rxBufIndex(0), txBufIndex(0), buf(buf) {}

    void init();
    void waitTxFree();
    bool isTxBusy();
    void sendData(uint8_t* data, uint16_t size);
    void setBaudrate(uint32_t baudrate);
    uint16_t getRxBufIndex();
    void setRxBufIndex(uint16_t index);
    uint16_t getTxBufIndex();
    void setTxBufIndex(uint16_t index);
    uint8_t* getBuf();
    void receiveEn();
    void receiveDis();
    void setRxDelay(uint16_t delay);
};

extern Uart1_t uart;
extern const uint32_t baud[];

#endif  // _UART_1_H_
