
#include "stm32l475_flash_cpp.h"
#include "stm32l475_rcc_cpp.h"
#include "uart1.h"

#include "dataflash.h"


df_data_t dataflash;

/**
 * @brief Read all dataflash
 */
void dataflash_t::readDataflash()
{
    for(uint16_t i = 0; i < sizeof(df_data_t); i++) {
        *(reinterpret_cast<uint8_t*>(&dataflash) + i) = *(reinterpret_cast<uint8_t*>(DATAFLASH_ADR_START) + i);
    }
}


/**
 * @brief Write all dataflash
 */
void dataflash_t::writeDataflash()
{
    flash_t::unlock();

    flash_t::erase_page(DATAFLASH_ADR_START);

    for(uint16_t i = 0; i < (sizeof(df_data_t) / sizeof(uint64_t)); i++) {
        flash_t::program_dword(DATAFLASH_ADR_START + (i * sizeof(uint64_t)), *(reinterpret_cast<uint64_t*>(&dataflash) + i));
    }

    flash_t::lock();
}


/**
 * @brief Init all non-volatile memory data
 */
void dataflash_t::initDataflash()
{
    // read all dataflash data
    dataflash_t::readDataflash();

    // set default uart baudrate
    if(dataflash.uartBaud == 0xFFFF) {
        dataflash.uartBaud = UART_BAUD_19200;
    }

    // set default uart receive delay
    if(dataflash.uartDelay == 0xFFFF) {
        dataflash.uartDelay = 20;
    }

    // write all EEPROM data
    dataflash_t::writeDataflash();
}
