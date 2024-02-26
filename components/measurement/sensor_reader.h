#ifndef MASTERTHESIS_SENSOR_READER_H
#define MASTERTHESIS_SENSOR_READER_H

#include <esp_adc/adc_oneshot.h>
#include "stdint.h"
#include "driver/gpio.h"


#define LED_PIN GPIO_NUM_13

// GPIO15
#define PIR_SENSOR_PIN GPIO_NUM_15

// GPIO27 D27
#define HALL_SENSOR_PIN GPIO_NUM_27

// GPIO34 A2
#define CO_SENSOR_ADC_CHANNEL ADC_CHANNEL_6

// GPIO36 A4
#define LEAKAGE_SENSOR_PIN GPIO_NUM_36

// GPIO37
#define ACCELEROMETER_INTR_PIN GPIO_NUM_37

#define BUTTON_PIN GPIO_NUM_38

// GPIO39 A3
#define ODOR_SENSOR_ADC_CHANNEL ADC_CHANNEL_3

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     20

#define AHT20_ADDR      0x38

#define LIS3DH_ADDR     0x18
#define LIS3DH_REG_CTRL1 0x20
#define LIS3DH_REG_CTRL2 0x21
#define LIS3DH_REG_CTRL3 0x22
#define LIS3DH_REG_CTRL4 0x23
#define LIS3DH_REG_CTRL5 0x24
#define LIS3DH_REG_REFERENCE 0x26
#define LIS3DH_REG_OUT_X_L 0x28
#define LIS3DH_REG_INT1_CFG 0x30
#define LIS3DH_REG_INT1_SRC 0x31
#define LIS3DH_REG_INT1_THS 0x32
#define LIS3DH_REG_INT1_DUR 0x33

extern adc_oneshot_unit_handle_t adc1_handle;

typedef struct Acceleration {
    float x;
    float y;
    float z;
} Acceleration;


void initButton(gpio_isr_t button_isr);

void initLED();

void set_led_level(uint8_t level);

int get_led_level();

void initADCs();

void initI2CDriver();

void initTemperatureSensor();

float readTemperatureSensor();

uint8_t lis3dh_read_register(uint8_t reg);

void lis3dh_init(uint8_t threshold);

Acceleration readAccelerometer();

void initLeakageSensor();

int readLeakageSensor();

void initPIRSensor();

int readPIRSensor();

void initHallSensor();

int readHallSensor();

uint32_t readCOSensor();

uint32_t readOdorSensor();

uint32_t calculate_odor_mean(int numValues);

uint32_t calculate_co_mean(int numValues);

//void calibrateAccelerometer(int numValues, Acceleration *mean, Acceleration *stdDev);

void calibrateTemperatureSensor(int numValues, float *mean);


#endif //MASTERTHESIS_SENSOR_READER_H
