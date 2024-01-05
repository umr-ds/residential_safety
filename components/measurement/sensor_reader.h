#ifndef MASTERTHESIS_SENSOR_READER_H
#define MASTERTHESIS_SENSOR_READER_H

#include <esp_adc/adc_oneshot.h>
#include "stdint.h"

// GPIO34 A2
#define CO_SENSOR_ADC_CHANNEL ADC_CHANNEL_6

// GPIO39 A3
#define ODOR_SENSOR_ADC_CHANNEL ADC_CHANNEL_3

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     20

#define AHT20_ADDR      0x38

#define LIS3DH_ADDR     0x18
#define LIS3DH_REG_CTRL1 0x20
#define LIS3DH_REG_CTRL4 0x23
#define LIS3DH_REG_OUT_X_L 0x28

extern adc_oneshot_unit_handle_t adc1_handle;

typedef struct Acceleration {
    float x;
    float y;
    float z;
} Acceleration;

void initSensors();

uint32_t readCOSensor();
uint32_t readOdorSensor();
float readTemperatureSensor();
Acceleration readAccelerometer();

void calibrateOdor(int numValues, float *mean, float *stdDev);
void calibrateCO(int numValues, float *mean, float *stdDev);
void calibrateTemperature(int numValues, float *mean, float *stdDev);
void calibrateAccelerometer(int numValues, Acceleration *mean, Acceleration *stdDev);

#endif //MASTERTHESIS_SENSOR_READER_H
