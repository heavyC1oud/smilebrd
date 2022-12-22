#ifndef _I2C_1_H_
#define _I2C_1_H_

typedef enum {
    I2C_1_ISR_PRIORITY = 1,
    I2C_1_SLAVE_ADDRESS = 0x48,
} I2C_1_param_t;

class i2c1_t
{
private:
    uint8_t data;

public:
    void init();
    void writeData(uint8_t addr, uint8_t* data, uint16_t count);
    void readData(uint8_t addr, uint8_t* data, uint16_t count);
    uint8_t getData();
    void setData(uint8_t data);
};

extern i2c1_t i2c1;

#endif  // _I2C_1_H_
