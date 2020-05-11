#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

void readRegister(uint8_t address, uint8_t *cmd, size_t cmd_size, uint8_t *data, size_t data_size);

void app_main(void)
{
    esp_err_t ret;

    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = GPIO_NUM_41;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num = GPIO_NUM_40;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = 400000;

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);

    uint8_t data;

    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x20 << 1) | I2C_MASTER_WRITE, true); // ack_en = true
    i2c_master_write(cmd, &data, 1, true);                            // ack_en = true
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, (50 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        printf("ESP OK\n");
    }
    int i = 0;
    while (1)
    {
        uint8_t error, address;
        for (address = 1; address < 127; address++)
        {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true); // ack_en = true
            i2c_master_write(cmd, &data, 1, true);                               // ack_en = true
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, (50 / portTICK_RATE_MS));
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK)
            {
                printf("Address is [0x%x]\n", address);
            }
        }
        printf("\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void readRegister(uint8_t address, uint8_t *cmd, size_t cmd_size, uint8_t *data, size_t data_size)
{
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);
    if (cmd_size > 0)
    {
        i2c_master_write_byte(_cmd, (address << 1) | I2C_MASTER_WRITE, true); // ack_en = true
        i2c_master_write(_cmd, cmd, cmd_size, true);                          // ack_en = true
        // repeat start
        i2c_master_start(_cmd);
    }
    i2c_master_write_byte(_cmd, (address << 1) | I2C_MASTER_READ, true); // ack_en = true
    if (data_size > 1)
    {
        i2c_master_read(_cmd, data, data_size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(_cmd, data + data_size - 1, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK);
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
}
