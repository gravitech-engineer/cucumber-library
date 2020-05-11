#include "HTS221Reg.h"
#include "HTS221.h"

static inline bool humidityReady(uint8_t data)
{
    return (data & 0x02);
}
static inline bool temperatureReady(uint8_t data)
{
    return (data & 0x01);
}

HTS221::HTS221(void) : _address(HTS221_ADDRESS)
{
    _temperature = 0.0;
    _humidity = 0.0;
}

bool HTS221::begin(void)
{
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

    data = readRegister(_address, WHO_AM_I);
    if (data == WHO_AM_I_RETURN)
    {
        if (activate())
        {
            storeCalibration();
            return true;
        }
    }

    return false;
}

bool HTS221::storeCalibration(void)
{
    uint8_t data;
    uint16_t tmp;

    for (int reg = CALIB_START; reg <= CALIB_END; reg++)
    {
        if ((reg != CALIB_START + 8) && (reg != CALIB_START + 9) && (reg != CALIB_START + 4))
        {

            data = readRegister(HTS221_ADDRESS, reg);

            switch (reg)
            {
            case CALIB_START:
                _h0_rH = data;
                break;
            case CALIB_START + 1:
                _h1_rH = data;
                break;
            case CALIB_START + 2:
                _T0_degC = data;
                break;
            case CALIB_START + 3:
                _T1_degC = data;
                break;

            case CALIB_START + 5:
                tmp = _T0_degC;
                _T0_degC = (data & 0x3) << 8;
                _T0_degC |= tmp;

                tmp = _T1_degC;
                _T1_degC = ((data & 0xC) >> 2) << 8;
                _T1_degC |= tmp;
                break;
            case CALIB_START + 6:
                _H0_T0 = data;
                break;
            case CALIB_START + 7:
                _H0_T0 |= data << 8;
                break;
            case CALIB_START + 0xA:
                _H1_T0 = data;
                break;
            case CALIB_START + 0xB:
                _H1_T0 |= data << 8;
                break;
            case CALIB_START + 0xC:
                _T0_OUT = data;
                break;
            case CALIB_START + 0xD:
                _T0_OUT |= data << 8;
                break;
            case CALIB_START + 0xE:
                _T1_OUT = data;
                break;
            case CALIB_START + 0xF:
                _T1_OUT |= data << 8;
                break;

            case CALIB_START + 8:
            case CALIB_START + 9:
            case CALIB_START + 4:
                //DO NOTHING
                break;

            // to cover any possible error
            default:
                return false;
            } /* switch */
        }     /* if */
    }         /* for */
    return true;
}

bool HTS221::activate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data |= POWER_UP;
    data |= ODR0_SET;
    writeRegister(_address, CTRL_REG1, data);

    return true;
}

bool HTS221::deactivate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data &= ~POWER_UP;
    writeRegister(_address, CTRL_REG1, data);
    return true;
}

bool HTS221::bduActivate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data |= BDU_SET;
    writeRegister(_address, CTRL_REG1, data);

    return true;
}

bool HTS221::bduDeactivate(void)
{
    uint8_t data;

    data = readRegister(_address, CTRL_REG1);
    data &= ~BDU_SET;
    writeRegister(_address, CTRL_REG1, data);
    return true;
}

const double
HTS221::readHumidity(void)
{
    uint8_t data = 0;
    uint16_t h_out = 0;
    double h_temp = 0.0;
    double hum = 0.0;

    data = readRegister(_address, STATUS_REG);

    if (data & HUMIDITY_READY)
    {
        data = readRegister(_address, HUMIDITY_H_REG);
        h_out = data << 8; // MSB
        data = readRegister(_address, HUMIDITY_L_REG);
        h_out |= data; // LSB

        // Decode Humidity
        hum = ((int16_t)(_h1_rH) - (int16_t)(_h0_rH)) / 2.0; // remove x2 multiple

        // Calculate humidity in decimal of grade centigrades i.e. 15.0 = 150.
        h_temp = (double)(((int16_t)h_out - (int16_t)_H0_T0) * hum) /
                 (double)((int16_t)_H1_T0 - (int16_t)_H0_T0);
        hum = (double)((int16_t)_h0_rH) / 2.0; // remove x2 multiple
        _humidity = (hum + h_temp);            // provide signed % measurement unit
    }
    return _humidity;
}

const double
HTS221::readTemperature(void)
{
    uint8_t data = 0;
    uint16_t t_out = 0;
    double t_temp = 0.0;
    double deg = 0.0;

    data = readRegister(_address, STATUS_REG);

    if (data & TEMPERATURE_READY)
    {

        data = readRegister(_address, TEMP_H_REG);
        t_out = data << 8; // MSB
        data = readRegister(_address, TEMP_L_REG);
        t_out |= data; // LSB

        // Decode Temperature
        deg = (double)((int16_t)(_T1_degC) - (int16_t)(_T0_degC)) / 8.0; // remove x8 multiple

        // Calculate Temperature in decimal of grade centigrades i.e. 15.0 = 150.
        t_temp = (double)(((int16_t)t_out - (int16_t)_T0_OUT) * deg) /
                 (double)((int16_t)_T1_OUT - (int16_t)_T0_OUT);
        deg = (double)((int16_t)_T0_degC) / 8.0; // remove x8 multiple
        _temperature = deg + t_temp;             // provide signed celsius measurement unit
    }

    return _temperature;
}

// Read a single uint8_t from addressToRead and return it as a uint8_t
uint8_t HTS221::readRegister(uint8_t slaveAddress, uint8_t regToRead)
{
   
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;
    uint8_t values;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);

    i2c_master_write_byte(_cmd, (slaveAddress << 1) | I2C_MASTER_WRITE, true); // ack_en = true
    i2c_master_write(_cmd, &regToRead, sizeof(regToRead), true);               // ack_en = true
    // repeat start
    i2c_master_start(_cmd);

    i2c_master_write_byte(_cmd, (slaveAddress << 1) | I2C_MASTER_READ, true); // ack_en = true

    i2c_master_read_byte(_cmd, &values, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK);
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
    return values;
}

// Writes a single uint8_t (dataToWrite) into regToWrite
bool HTS221::writeRegister(uint8_t slaveAddress, uint8_t regToWrite, uint8_t dataToWrite)
{
    
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    uint8_t data[] = {regToWrite, dataToWrite};

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);

    i2c_master_write_byte(_cmd, (slaveAddress << 1) | I2C_MASTER_WRITE, true); // ack_en = true
    i2c_master_write(_cmd, data, sizeof(data), true);                          // ack_en = true
    // repeat start
    i2c_master_start(_cmd);

    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
    if (ret == ESP_OK)
        return true;
    else
        return false;
}

// HTS221 smeHumidity;
