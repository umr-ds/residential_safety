#include <stdio.h>
#include <esp_timer.h>
#include "driver/i2c.h"
#include "math.h"
#include "main.h"

void initI2CDriver() {
    i2c_config_t i2c_cfg = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .clk_flags = 0,
    };
    i2c_cfg.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

void initAccelerometer() {
    // Data rate: 100Hz, Low-Power disabled, X,Y,Z Axes enabled
    uint8_t writeBuf[2] = {LIS3DH_REG_CTRL1, 0x57};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL2;
    writeBuf[1] = 0x00; // High pass filter normal mode (reset by reading REF register), enabled for INT1
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL4;
    writeBuf[1] = 0x00; // Continuous update, Little Endian, 2G-Scale, high-resoultion disabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

}


Acceleration readAccelerometer() {
    uint8_t data[6];
    Acceleration acceleration;
    uint8_t writeBuf[1] = {LIS3DH_REG_OUT_X_L | 0x80,};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, writeBuf, sizeof(writeBuf), 1000 / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_NUM_0, LIS3DH_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);

    // Convert raw data to 16-bit signed integers
    // and shift by 4 because of 12-bit data
    int16_t x, y, z;
    x = ((int16_t) data[1] << 8) + (uint16_t) data[0];
    x = x >> 6;
    y = ((int16_t) data[3] << 8) + (uint16_t) data[2];
    y = y >> 6;
    z = ((int16_t) data[5] << 8) + (uint16_t) data[4];
    z = z >> 6;
    float sensitivity = 0.004;
    acceleration.x = (float) x * sensitivity;
    acceleration.y = (float) y * sensitivity;
    acceleration.z = (float) z * sensitivity;

    return acceleration;
}


void app_main(void)
{
    initI2CDriver();
    initAccelerometer();
    printf("Time,X,Y,Z\n");
    while(1){
        uint64_t time = esp_timer_get_time();
        Acceleration accel = readAccelerometer();
        printf("%llu,%f,%f,%f\n", time/1000, accel.x, accel.y, accel.z);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
