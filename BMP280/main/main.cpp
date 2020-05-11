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
                printf(" degC\t");
            }
            else
            {
                printf("Error.");
            }
        }
        cout << "hello world" << endl;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
       
    }
}
