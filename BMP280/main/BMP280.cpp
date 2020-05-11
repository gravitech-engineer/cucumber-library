#include "BMP280.h"
#include <stdio.h>
#include <math.h>

BMP280::BMP280()
{
    //do nothing
}
/*
*	Initialize library and coefficient for measurements
*/
char BMP280::begin(gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clk_speed)
{
    i2c_config_t i2c_conf;

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = sdaPin;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_io_num = sclPin;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = clk_speed;

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);

    return (readCalibration());
}

char BMP280::begin()
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
    return (readCalibration());
}

// The BMP280 includes factory calibration data stored on the device.
// Each device has different numbers, these must be retrieved and
// used in the calculations when taking measurements.

// Retrieve calibration data from device:
char BMP280::readCalibration()
{
    if (
        readUInt(0x88, dig_T1) &&
        readInt(0x8A, dig_T2) &&
        readInt(0x8C, dig_T3) &&
        readUInt(0x8E, dig_P1) &&
        readInt(0x90, dig_P2) &&
        readInt(0x92, dig_P3) &&
        readInt(0x94, dig_P4) &&
        readInt(0x96, dig_P5) &&
        readInt(0x98, dig_P6) &&
        readInt(0x9A, dig_P7) &&
        readInt(0x9C, dig_P8) &&
        readInt(0x9E, dig_P9))
    {
#ifdef _debugSerial
        Serial.print("dig_T1=");
        Serial.println(dig_T1, 2);
        Serial.print("dig_T2=");
        Serial.println(dig_T2, 2);
        Serial.print("dig_T3=");
        Serial.println(dig_T3, 2);
        Serial.print("dig_P1=");
        Serial.println(dig_P1, 2);
        Serial.print("dig_P2=");
        Serial.println(dig_P2, 2);
        Serial.print("dig_P3=");
        Serial.println(dig_P3, 2);
        Serial.print("dig_P4=");
        Serial.println(dig_P4, 2);
        Serial.print("dig_P5=");
        Serial.println(dig_P5, 2);
        Serial.print("dig_P6=");
        Serial.println(dig_P6, 2);
        Serial.print("dig_P7=");
        Serial.println(dig_P7, 2);
        Serial.print("dig_P8=");
        Serial.println(dig_P8, 2);
        Serial.print("dig_P9=");
        Serial.println(dig_P9, 2);
#endif
#ifdef _debugTestData
        dig_T1 = 27504.0;
        dig_T2 = 26435.0;
        dig_T3 = -1000.0;
        dig_P1 = 36477.0;
        dig_P2 = -10685.0;
        dig_P3 = 3024.0;
        dig_P4 = 2855.0;
        dig_P5 = 140.0;
        dig_P6 = -7.0;
        dig_P7 = 15500.0;
        dig_P8 = -14600.0;
        dig_P9 = 6000.0;
        Serial.print("dig_T1=");
        Serial.println(dig_T1, 2);
        Serial.print("dig_T2=");
        Serial.println(dig_T2, 2);
        Serial.print("dig_T3=");
        Serial.println(dig_T3, 2);
        Serial.print("dig_P1=");
        Serial.println(dig_P1, 2);
        Serial.print("dig_P2=");
        Serial.println(dig_P2, 2);
        Serial.print("dig_P3=");
        Serial.println(dig_P3, 2);
        Serial.print("dig_P4=");
        Serial.println(dig_P4, 2);
        Serial.print("dig_P5=");
        Serial.println(dig_P5, 2);
        Serial.print("dig_P6=");
        Serial.println(dig_P6, 2);
        Serial.print("dig_P7=");
        Serial.println(dig_P7, 2);
        Serial.print("dig_P8=");
        Serial.println(dig_P8, 2);
        Serial.print("dig_P9=");
        Serial.println(dig_P9, 2);
#endif
        return (1);
    }
    else
        return (0);
}

/*
**	Read a signed integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value   = external variable to store data (function modifies value)
*/
char BMP280::readInt(char address, double &value)

{
    unsigned char data[2]; //char is 4bit,1byte

    data[0] = address;
    if (readBytes(data, 2))
    {
        value = (double)(int16_t)(((unsigned int)data[1] << 8) | (unsigned int)data[0]); //
        return (1);
    }
    value = 0;
    return (0);
}
/* 
**	Read an unsigned integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value 	 = external variable to store data (function modifies value)
*/

char BMP280::readUInt(char address, double &value)
{
    unsigned char data[2]; //4bit
    data[0] = address;
    if (readBytes(data, 2))
    {
        value = (double)(unsigned int)(((unsigned int)data[1] << 8) | (unsigned int)data[0]);
        return (1);
    }
    value = 0;
    return (0);
}
/*
** Read an array of bytes from device
** @param : value  = external array to hold data. Put starting register in values[0].
** @param : length = number of bytes to read
*/

char BMP280::readBytes(unsigned char *values, char length)
{
    char x;

   i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);
    if (1 > 0)
    {
        i2c_master_write_byte(_cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true); // ack_en = true
        i2c_master_write(_cmd, values, 1, true);                                  // ack_en = true
        // repeat start
        i2c_master_start(_cmd);
    }
    i2c_master_write_byte(_cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true); // ack_en = true
    if (length > 1)
    {
        i2c_master_read(_cmd, values, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(_cmd, values + length - 1, I2C_MASTER_NACK); //I2C_MASTER_LAST_NACK);
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
    return (1);
}
/*
** Write an array of bytes to device
** @param : values = external array of data to write. Put starting register in values[0].
** @param : length = number of bytes to write
*/
char BMP280::writeBytes(unsigned char *values, size_t length)
{
    i2c_cmd_handle_t _cmd;
    esp_err_t ret;

    _cmd = i2c_cmd_link_create();
    i2c_master_start(_cmd);
    if (length > 0)
    {
        i2c_master_write_byte(_cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true); // ack_en = true
        i2c_master_write(_cmd, values, length, true);                             // ack_en = true
        // repeat start
        i2c_master_start(_cmd);
    }
    i2c_master_stop(_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, _cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_cmd);
    return (1);
}

short BMP280::getOversampling(void)
{
    return oversampling;
}

char BMP280::setOversampling(short oss)
{
    oversampling = oss;
    return (1);
}
/*
**	Begin a measurement cycle.
** Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
** @returns : delay in ms to wait, or 0 if I2C error.
*/
char BMP280::startMeasurment(void)

{
    unsigned char data[2], result, delay;

    data[0] = BMP280_REG_CONTROL;

    switch (oversampling)
    {
    case 0:
        data[1] = BMP280_COMMAND_PRESSURE0;
        oversampling_t = 1;
        delay = 8;
        break;
    case 1:
        data[1] = BMP280_COMMAND_PRESSURE1;
        oversampling_t = 1;
        delay = 10;
        break;
    case 2:
        data[1] = BMP280_COMMAND_PRESSURE2;
        oversampling_t = 1;
        delay = 15;
        break;
    case 3:
        data[1] = BMP280_COMMAND_PRESSURE3;
        oversampling_t = 1;
        delay = 24;
        break;
    case 4:
        data[1] = BMP280_COMMAND_PRESSURE4;
        oversampling_t = 1;
        delay = 45;
        break;
    case 16:
        data[1] = BMP280_COMMAND_OVERSAMPLING_MAX;
        oversampling_t = 1;
        delay = 80; //I cannot find any data about timings in datasheet for x16 pressure and x16 temeprature oversampling
        break;      //I guess this is enough time (maybe it can be even smaller ~60ms)
    default:
        data[1] = BMP280_COMMAND_PRESSURE0;
        delay = 9;
        break;
    }
    result = writeBytes(data, 2);
    if (result)         // good write?
        return (delay); // return the delay in ms (rounded up) to wait before retrieving data
    else
        return (0); // or return 0 if there was a problem communicating with the BMP
}

/*
**	Get the uncalibrated pressure and temperature value.
**  @param : uP = stores the uncalibrated pressure value.(20bit)
**  @param : uT = stores the uncalibrated temperature value.(20bit)
*/
char BMP280::getUnPT(double &uP, double &uT)
{
    unsigned char data[6];
    char result;

    data[0] = BMP280_REG_RESULT_PRESSURE; //0xF7

    result = readBytes(data, 6); // 0xF7; xF8, 0xF9, 0xFA, 0xFB, 0xFC
    if (result)                  // good read
    {
        double factor = pow(2, 4);
        uP = (double)(data[0] * 4096 + data[1] * 16 + data[2] / 16); //20bit UP
        uT = (double)(data[3] * 4096 + data[4] * 16 + data[5] / 16); //20bit UT
#ifdef _debugSerial
        Serial.print(uT);
        Serial.print(" ");
        Serial.println(uP);
#endif
#ifdef _debugTestData
        uT = 519888.0;
        uP = 415148.0;
        Serial.print(uT);
        Serial.print(" ");
        Serial.println(uP);
#endif
    }
    return (result);
}
/*
** Retrieve temperature and pressure.
** @param : T = stores the temperature value in degC.
** @param : P = stores the pressure value in mBar.
*/
char BMP280::getTemperatureAndPressure(double &T, double &P)
{
    double uT;
    double uP;
    char result = getUnPT(uP, uT);
    if (result != 0)
    {
        // calculate the temperature
        result = calcTemperature(T, uT);
        if (result)
        {
            // calculate the pressure
            result = calcPressure(P, uP);
            if (result)
                return (1);
            else
                error = 3; // pressure error ;
            return (9);
        }
        else
            error = 2; // temperature error ;
    }
    else
        error = 1;

    return (9);
}
/*
** temperature calculation
** @param : T  = stores the temperature value after calculation.
** @param : uT = the uncalibrated temperature value.
*/
char BMP280::calcTemperature(double &T, double &adc_T)
//
{
    //Serial.print("adc_T = "); Serial.println(adc_T,DEC);

    double var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2;
    double var2 = ((adc_T / 131072.0 - dig_T1 / 8192.0) * (adc_T / 131072.0 - dig_T1 / 8192.0)) * dig_T3;
    t_fine = var1 + var2;
    T = (var1 + var2) / 5120.0;
#ifdef _debugSerial
    Serial.print(var1);
    Serial.print(" ");
    Serial.print(var2);
    Serial.print(" ");
    Serial.print(t_fine);
    Serial.print(" ");
    Serial.println(T);
#endif

    if (T > 100 || T < -100)
        return 0;

    return (1);
}
/*
**	Pressure calculation from uncalibrated pressure value.
**  @param : P  = stores the pressure value.
**  @param : uP = uncalibrated pressure value. 
*/
char BMP280::calcPressure(double &P, double uP)
{
    //char result;
    double var1, var2;

    var1 = (t_fine / 2.0) - 64000.0;
#ifdef _debugSerial
    Serial.print("var1 = ");
    Serial.println(var1, 2);
#endif
    var2 = var1 * (var1 * dig_P6 / 32768.0); //not overflow
#ifdef _debugSerial
    Serial.print("var2 = ");
    Serial.println(var2, 2);
#endif
    var2 = var2 + (var1 * dig_P5 * 2.0); //overflow
#ifdef _debugSerial
    Serial.print("var2 = ");
    Serial.println(var2, 2);
#endif

    var2 = (var2 / 4.0) + ((dig_P4)*65536.0);
#ifdef _debugSerial
    Serial.print("var2 = ");
    Serial.println(var2, 2);
#endif

    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
#ifdef _debugSerial
    Serial.print("var1 = ");
    Serial.println(var1, 2);
#endif
    var1 = (1.0 + var1 / 32768.0) * dig_P1;
#ifdef _debugSerial
    Serial.print("var1 = ");
    Serial.println(var1, 2);
#endif

    P = 1048576.0 - uP;
#ifdef _debugSerial
    Serial.print("p = ");
    Serial.println(p, 2);
#endif

    P = (P - (var2 / 4096.0)) * 6250.0 / var1; //overflow
#ifdef _debugSerial
    Serial.print("p = ");
    Serial.println(p, 2);
#endif

    var1 = dig_P9 * P * P / 2147483648.0; //overflow
#ifdef _debugSerial
    Serial.print("var1 = ");
    Serial.println(var1, 2);
#endif

    var2 = P * dig_P8 / 32768.0;
#ifdef _debugSerial
    Serial.print("var2 = ");
    Serial.println(var2, 2);
#endif
    P = P + (var1 + var2 + dig_P7) / 16.0;
#ifdef _debugSerial
    Serial.print("p = ");
    Serial.println(p, 2);
#endif

    P = P / 100.0;

    if (P > 1200.0 || P < 800.0)
        return (0);
    return (1);
}

double BMP280::sealevel(double P, double A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
    return (P / pow(1 - (A / 44330.0), 5.255));
}

double BMP280::altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
    return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}

char BMP280::getError(void)
// If any library command fails, you can retrieve an extended
// error code using this command. Errors are from the wire library:
// 0 = Success
// 1 = Data too long to fit in transmit buffer
// 2 = Received NACK on transmit of address
// 3 = Received NACK on transmit of data
// 4 = Other error
{
    return (error);
}
