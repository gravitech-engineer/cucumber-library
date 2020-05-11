#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "MPU6050.h"

extern "C"
{
  void app_main();
}

void checkSettings();
MPU6050 mpu;
mpu6050_dlpf_t dlpf;

void app_main(void)
{
  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    printf("Could not find a valid MPU6050 sensor, check wiring!");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  mpu.calibrateGyro(15);
  mpu.setThreshold(3);
  //mpu.setDLPFMode(dlpf);
  checkSettings();

  while (1)
  {
    // Vector rawGyro = mpu.readRawGyro();
    Vector NGyro = mpu.readNormalizeGyro();
    Vector NAcc = mpu.readNormalizeAccel();

    printf("Read Gyroscope\n");
    printf(" XAxis = %lf \n", NGyro.XAxis);
    printf(" YAxis = %lf \n", NGyro.YAxis);
    printf(" ZAxis = %lf \n", NGyro.ZAxis);
    printf("\n");
    printf("Read Accelerometer\n");
    printf(" XAxis = %lf \n", NAcc.XAxis);
    printf(" YAxis = %lf \n", NAcc.YAxis);
    printf(" ZAxis = %lf \n", NAcc.ZAxis);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void checkSettings()
{

  printf(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
  case MPU6050_CLOCK_KEEP_RESET:
    printf("Stops the clock and keeps the timing generator in reset\n");
    break;
  case MPU6050_CLOCK_EXTERNAL_19MHZ:
    printf("PLL with external 19.2MHz reference\n");
    break;
  case MPU6050_CLOCK_EXTERNAL_32KHZ:
    printf("PLL with external 32.768kHz reference\n");
    break;
  case MPU6050_CLOCK_PLL_ZGYRO:
    printf("PLL with Z axis gyroscope reference\n");
    break;
  case MPU6050_CLOCK_PLL_YGYRO:
    printf("PLL with Y axis gyroscope reference\n");
    break;
  case MPU6050_CLOCK_PLL_XGYRO:
    printf("PLL with X axis gyroscope reference\n");
    break;
  case MPU6050_CLOCK_INTERNAL_8MHZ:
    printf("Internal 8MHz oscillator\n");
    break;
  }

  printf(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
  case MPU6050_SCALE_2000DPS:
    printf("2000 dps\n");
    break;
  case MPU6050_SCALE_1000DPS:
    printf("1000 dps\n");
    break;
  case MPU6050_SCALE_500DPS:
    printf("500 dps\n");
    break;
  case MPU6050_SCALE_250DPS:
    printf("250 dps\n");
    break;
  }

  printf(" * Gyroscope offsets: %d/%d/%d \n", mpu.getGyroOffsetX(), mpu.getGyroOffsetY(), mpu.getGyroOffsetZ());
}