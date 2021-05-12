
#include "basis.h"
#include "board.h"
#include "stm32l475_pin_cpp.h"
#include "stm32l475_i2c_cpp.h"
#include "i2c1.h"


// I2C low level routines
i2c_ll_t<I2C_1> i2c1_ll;

// I2C instance
i2c1_t i2c1;


void i2c1_t::init()
{
    this->data = 0;

    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOB);
    pin_i2c1_scl.Mode(ALT_OUTPUT_OD_PULLUP);
    pin_i2c1_scl.Alternate(ALT_FUNC4);
    pin_i2c1_sda.Mode(ALT_OUTPUT_OD_PULLUP);
    pin_i2c1_sda.Alternate(ALT_FUNC4);

    // I2C settings
    i2c1_ll.init(I2C_1_SLAVE_ADDRESS, I2C_1_ISR_PRIORITY);
    i2c1_ll.en_addr_irq();
}



/**
 * @brief I2C write data
 *
 * @param addr: slave address
 * @param data: data to write
 * @param count: size of data
 */
void i2c1_t::writeData(uint8_t addr, uint8_t* data, uint16_t count)
{
    // set slave address
    i2c1_ll.set_addr_master(addr);

    // set number bytes to send
    i2c1_ll.set_nbytes(count);

    // set transfer direction to write
    i2c1_ll.set_dir_master(I2C_MASTER_WRITE);

    // generate START condition
    i2c1_ll.start();

    if(count == 0) {
        return;
    }

    // wait TXE flag or NACK flag Is Set
    while((!(i2c1_ll.is_txe_flag())) && (!(i2c1_ll.is_nack_flag()))) {
        if(i2c1_ll.is_nack_flag()) {
            i2c1_ll.nack_flag_clear();
            return;
        }
    }

    // transmit Data
    for(uint16_t i = 0; i < count; i++)
    {
        //	write data
        i2c1_ll.putch(data[i]);

        if(i != (count - 1)) {
            // wait TXE flag or NACK flag Is Set
            while((!(i2c1_ll.is_txe_flag())) && (!(i2c1_ll.is_nack_flag()))) {
                if(i2c1_ll.is_nack_flag()) {
                    i2c1_ll.nack_flag_clear();
                    return;
                }
            }
        }
    }

    delayMs(1);
}


/**
 * @brief I2C read data
 *
 * @param addr: slave address
 * @param data: data to read
 * @param count: size of data
 */
void i2c1_t::readData(uint8_t addr, uint8_t* data, uint16_t count)
{
    // set slave address
    i2c1_ll.set_addr_master(addr);

    // set number bytes to send
    i2c1_ll.set_nbytes(count);

    // set transfer direction to read
    i2c1_ll.set_dir_master(I2C_MASTER_READ);

    // generate START condition
    i2c1_ll.start();

    // wait TXE flag or NACK flag Is Set
    while((!(i2c1_ll.is_txe_flag())) && (!(i2c1_ll.is_nack_flag()))) {
        if(i2c1_ll.is_nack_flag()) {
            i2c1_ll.nack_flag_clear();
            return;
        }
    }

    // receive Data
    for(uint16_t i = 0; i < count; i++)
    {
        // wait RXNE flag Is Set
        while(i2c1_ll.is_rxne_flag() == 0);

        // read data
        data[i] = i2c1_ll.getch();
    }
}


/**
 * @brief Get data value
 * @return: data value
 */
uint8_t i2c1_t::getData()
{
    return this->data;
}


/**
 * @brief Set data value
 * @param data - data value
 */
void i2c1_t::setData(uint8_t data)
{
    this->data = data;
}


/**
 * @brief I2C1 IRQ
 */
extern "C" void I2C1_EV_IRQHandler(void)
{
    if(i2c1_ll.is_addr_match_flag()) {
        // Write transfer, slave enters receiver mode
        if(i2c1_ll.is_dir_flag() == I2C_MASTER_WRITE) {
            // clear irq flag
            i2c1_ll.addr_flag_clear();
        }
        // Read transfer, slave enters transmitter mode
        else {
            // wait transmit register empty
            while(i2c1_ll.is_txe_flag() == 0);

            // write transmit data
            i2c1_ll.putch(i2c1.getData());

            // clear irq flag
            i2c1_ll.addr_flag_clear();
        }
    }
}
