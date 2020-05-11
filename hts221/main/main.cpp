#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "HTS221.h"

extern "C"
{
    void app_main();
}

HTS221 smeHumidity;

void app_main(void)
{
    smeHumidity.begin();

    int i = 0;
    while (1)
    {
        double data = 0;

        data = smeHumidity.readHumidity();
        printf("readHumidity! [%.2lf]\n", data);

        data = smeHumidity.readTemperature();
        printf("readTemperature [%.2lf]\n", data);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
