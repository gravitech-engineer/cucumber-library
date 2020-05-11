#include "Wire.h"

TwoWire::TwoWire()
{
}

TwoWire::~TwoWire()
{
    i2c_driver_delete(I2C_NUM_0);
}

void TwoWire::begin(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed)
{
    i2c_config_t i2c_conf;

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = sda_pin;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num = scl_pin;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = clk_speed;

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
}

bool TwoWire::detect(uint8_t _address)
{
    i2c_cmd_handle_t cmd;
    esp_err_t ret;
    uint8_t error;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true); // ack_en = true
    i2c_master_write(cmd, &error, 1, true);                               // ack_en = true
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, (50 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        // printf("Address is [0x%x]\n", address);
        return true;
    }
    else
    {
        return false;
    }
}

void TwoWire::readRegister(uint8_t _address, uint8_t *data, size_t data_size, uint8_t *values, size_t value_size)
{
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);
    if (data_size > 0)
    {
        i2c_master_write_byte(_cmd, (_address << 1) | I2C_MASTER_WRITE, true); // ack_en = true
        i2c_master_write(_cmd, data, data_size, true);                         // ack_en = true
        // repeat start
        i2c_master_start(_cmd);
    }
    i2c_master_write_byte(_cmd, (_address << 1) | I2C_MASTER_READ, true); // ack_en = true
    if (value_size > 1)
    {
        i2c_master_read(_cmd, values, value_size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(_cmd, values + value_size - 1, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK);
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
}

void writeRegister(uint8_t _address, uint8_t *data, size_t data_size)
{
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);
    if (data_size > 0)
    {
        i2c_master_write_byte(_cmd, (_address << 1) | I2C_MASTER_WRITE, true); // ack_en = true
        i2c_master_write(_cmd, data, data_size, true);                         // ack_en = true
        // repeat start
        i2c_master_start(_cmd);
    }
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
}

TwoWire Wire = TwoWire();
