
#ifndef _DATAFLASH_H_
#define _DATAFLASH_H_

#include <cstdint>


#define DATAFLASH_ADR_START 0x080FF800  //  page 511, 1 MB dual bank organization

typedef struct {
        uint16_t uartBaud;      // UART baudrate according to table:
                                //      0: 1200  | 1: 2400  | 2: 4800  | 3: 9600
                                //      4: 19200 | 5: 38400 | 6: 57600 | 7: 115200
        uint16_t uartDelay;     // UART answer delay in milliseconds
        uint32_t reserved0;
} df_data_t;
static_assert(!(sizeof(df_data_t) % sizeof(uint64_t)), "data structure must be aligned to double word");

class dataflash_t
{
public:
    static void readDataflash();
    static void writeDataflash();
    static void initDataflash();
};

extern df_data_t dataflash;


#endif  // _DATAFLASH_H_
