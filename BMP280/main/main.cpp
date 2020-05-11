#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "Wire.h"
#include <iostream>
#include "BMP280.h"
#include <string.h>

using namespace std;

extern "C"
{
    void app_main();
}

#define P0 1013.25
BMP280 bmp;

void app_main(void)
{
    Wire.begin(GPIO_NUM_12, GPIO_NUM_13, 400000);
    if (Wire.detect(0x76))
    {
        printf("Have address 0x76\n");
    }
    else
    {
        printf("no device 0x76");
    }
    uint8_t address;
    for (address = 1; address < 127; address++)
    {
        if (Wire.detect(address))
        {
            printf("Have address : [0x%x]\n", address);
        }
    }
    Wire.~TwoWire();

    uint8_t readID = 0x00, reg = 0xD0;
    // Wire.readRegister(0x76, &reg, 1, &readID, 1);
    printf("Have address : [0x%x][0x%x]\n", reg, readID);
    if (!bmp.begin(GPIO_NUM_12, GPIO_NUM_13, 400000))
    {
        printf("no device 0x76");
    }
    bmp.setOversampling(4);
    while (1)
    {
        double T, P;
        char result = bmp.startMeasurment();

        if (result != 0)
        {
            result = bmp.getTemperatureAndPressure(T, P);

            if (result != 0)
            {
                double A = bmp.altitude(P, P0);

                printf("T = \t%.2lf degC\tP = \t%.2lf mBar\tA = \t%.2lf m\n", T, P, A);
                // printf("T = \t%.2lf degC\tP = \t%.2lf mBar\n", T, P);

                // printf(T, 2);
                printf(" degC\t");
                // printf("P = \t");
                // printf(P, 2);
                // printf(" mBar\t");
                // printf("A = \t");
                // printf(A, 2);
                // printf(" m");
            }
            else
            {
                printf("Error.");
            }
        }
        // uint8_t command = 0x00;
        // uint8_t byte_read[2];
        // readRegister(0x4D, &command, sizeof(command), byte_read, sizeof(byte_read));
        // float temp = 0.0;
        // temp += (int)(byte_read[0] << 1);
        // if (byte_read[1] & 0b10000000)
        //     temp += 1.0;
        // if (byte_read[1] & 0b01000000)
        //     temp += 0.5;
        // if (byte_read[1] & 0b00100000)
        //     temp += 0.25;
        // if (byte_read[0] & 0b10000000)
        //     temp *= -1.0;
        // printf("LM73 : [%0.2f]\n", temp);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // printf("Hello World\n");
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // uint8_t address;
        // for (address = 1; address < 127; address++)
        // {
        //     if (detect(address))
        //     {
        //         printf("Have address : [0x%x]\n", address);
        //     }
        // }
        // printf("\n");
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        cout << "hello world" << endl;
        // readRegister(0x76, &reg, 1, &readID, 1);
        // printf("Have address : [0x%x]\n", readID);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // try
        // {
        //     /* code */
        //     // delete first;
        // }
        // catch (const std::exception &e)
        // {
        //     std::cerr << e.what() << '\n';
        // }
    }
}
