
#include "basis.h"
#include "board.h"
#include "stm32l475_pin_cpp.h"
#include "stm32l475_tsc_cpp.h"
#include "keyboard.h"


// keyboard instance
keyb_t keyb;

/**
 * @brief Keyboard initialization
 */
void keyb_t::init()
{
    this->sensor_value = 0;

    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOB);
    rcc_t::ahb2_periph_on(RCC_AHB2_PERIPH_GPIOC);

    // group 1 io1 sample pin
    pin_tsc_g1_io1_sample.Mode(ALT_OUTPUT_OD);
    pin_tsc_g1_io1_sample.Alternate(ALT_FUNC9);
    // group 1 io2 sense pin
    pin_tsc_g1_io2_sense.Mode(ALT_OUTPUT);
    pin_tsc_g1_io2_sense.Alternate(ALT_FUNC9);
    // group 1 io3 sense pin
    pin_tsc_g1_io3_sense.Mode(ALT_OUTPUT);
    pin_tsc_g1_io3_sense.Alternate(ALT_FUNC9);
    // group 1 io4 sense pin
    pin_tsc_g1_io4_sense.Mode(ALT_OUTPUT);
    pin_tsc_g1_io4_sense.Alternate(ALT_FUNC9);

    // group 4 io1 sample pin
    pin_tsc_g4_io1_sample.Mode(ALT_OUTPUT_OD);
    pin_tsc_g4_io1_sample.Alternate(ALT_FUNC9);
    // group 4 io2 sense pin
    pin_tsc_g4_io2_sense.Mode(ALT_OUTPUT);
    pin_tsc_g4_io2_sense.Alternate(ALT_FUNC9);
    // group 4 io3 sense pin
    pin_tsc_g4_io3_sense.Mode(ALT_OUTPUT);
    pin_tsc_g4_io3_sense.Alternate(ALT_FUNC9);
    // group 4 io4 sense pin
    pin_tsc_g4_io4_sense.Mode(ALT_OUTPUT);
    pin_tsc_g4_io4_sense.Alternate(ALT_FUNC9);

    tsc_t::init();

    // enable groups
    tsc_t::en_group_acq(TSC_GROUP_1);
    tsc_t::en_group_acq(TSC_GROUP_4);

    delayMs(1);
}


/**
 * @brief Poll
 *        poll one sensor from each group
 *
 * @param io - touch sensing controller i/o pin
 * @param group1_val - pointer to variable containing group 1 counter value
 * @param group4_val - pointer to variable containing group 4 counter value
 */
void keyb_t::poll(tsc_io_no_t io, uint32_t* group1_val, uint32_t* group4_val)
{
    // set sampling pins
    tsc_t::en_sampling(TSC_GROUP_1, TSC_IO_1);
    tsc_t::en_sampling(TSC_GROUP_4, TSC_IO_1);

    // enable group 1 sense pin
    tsc_t::en_sensing(TSC_GROUP_1, io);
    // enable group 4 sense pin
    tsc_t::en_sensing(TSC_GROUP_4, io);

    // start acquition
    tsc_t::start();

    // wait end of acquition
    while((tsc_t::is_group_acq(TSC_GROUP_1) && tsc_t::is_group_acq(TSC_GROUP_4)) == 0);

    tsc_t::dis_all_sensing();
    tsc_t::dis_all_sampling();

    // write calibration data
    *group1_val = tsc_t::get_counter(TSC_GROUP_1);
    *group4_val = tsc_t::get_counter(TSC_GROUP_4);

    // poll debounce
    delayMs(1);
}


/**
 * @brief Calibrate buttons
 *        write calibrate value, whitch is
 *        number of samples for empty sensor
 */
void keyb_t::calibrate()
{
    // calibrate button 1 and 4
    this->poll(TSC_IO_2, &(this->calibrate_value[KEYB_BUT_1]), &(this->calibrate_value[KEYB_BUT_4]));

    // calibrate button 2 and 5
    this->poll(TSC_IO_3, &(this->calibrate_value[KEYB_BUT_2]), &(this->calibrate_value[KEYB_BUT_5]));

    // calibrate button 3 and 6
    this->poll(TSC_IO_4, &(this->calibrate_value[KEYB_BUT_3]), &(this->calibrate_value[KEYB_BUT_6]));

    // if calculated calibrate value too small, set minimal calibrate value
    for(uint8_t i = 0; i < 6; i++) {
        if((this->calibrate_value[i] - KEYB_SENSING_LIMIT) > tsc_t::get_mcv()) {
            this->calibrate_value[i] = KEYB_SENSING_LIMIT + 2;
        }
    }
}


/**
 * @brief Handle sensors
 *        touching the sensor increase its capacity
 *        therefore the number or charging cycles of the
 *        sample capasitor decreases, then lower poll value
 *        means touching the sensor
 */
void keyb_t::handle()
{
    uint32_t sensorP1 = 0;
    uint32_t sensorP2 = 0;

    // poll button 1 and 4
    this->poll(TSC_IO_2, &sensorP1, &sensorP2);

    sensorP1 < (this->calibrate_value[KEYB_BUT_1] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_1) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_1);

    sensorP2 < (this->calibrate_value[KEYB_BUT_4] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_4) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_4);

    // poll button 2 and 5
    this->poll(TSC_IO_3, &sensorP1, &sensorP2);

    sensorP1 < (this->calibrate_value[KEYB_BUT_2] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_2) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_2);

    sensorP2 < (this->calibrate_value[KEYB_BUT_5] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_5) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_5);

    // poll button 3 and 6
    this->poll(TSC_IO_4, &sensorP1, &sensorP2);

    sensorP1 < (this->calibrate_value[KEYB_BUT_3] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_3) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_3);

    sensorP2 < (this->calibrate_value[KEYB_BUT_6] - KEYB_SENSING_LIMIT) ? this->sensor_value |= (1 << KEYB_BUT_6) :
                                                                          this->sensor_value &= ~(1 << KEYB_BUT_6);
}


/**
 * @brief Get sensor value
 *
 * @return - sensor state, true - action, false - no action
 */
bool keyb_t::getSensor(keyb_but_num_t num)
{
    return ((this->sensor_value & (1 << num)) != 0);
}


/**
 * @brief Get all sensor values
 *
 * @return - variable with all sensors state
 */
uint8_t keyb_t::getSensors()
{
    return this->sensor_value;
}


/**
 * @brief TSC IRQ
 */
extern "C" void TSC_IRQHandler(void)
{

}
