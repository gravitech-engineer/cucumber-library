#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

class TwoWire
{
public:
    i2c_config_t i2c_conf;

    TwoWire();
    ~TwoWire();
    void begin(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed);

    bool detect(uint8_t _address);

    void readRegister(uint8_t _address, uint8_t *data, size_t data_size, uint8_t *values, size_t value_size);

    void writeRegister(uint8_t _address, uint8_t *data, size_t data_size);
};
extern TwoWire Wire;
