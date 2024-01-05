#include "sensor_reader.h"
#include "driver/i2c.h"
#include "math.h"

adc_oneshot_unit_handle_t adc1_handle;

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
    // Configure LIS3DH
    //uint8_t writeBuf[2] = {LIS3DH_REG_CTRL1, 0x07};
    uint8_t writeBuf[2] = {LIS3DH_REG_CTRL1, 0x27};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
    writeBuf[0] = LIS3DH_REG_CTRL4;
    writeBuf[1] = 0x98;
    //writeBuf[1] = 0x20;
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
}

void initTemperatureSensor() {
    uint8_t writeBuf[3] = {0xBE, 0x08, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, AHT20_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
    //vTaskDelay(10 / portTICK_PERIOD_MS);
}

void initSensors() {


    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));
    /// Init ADCs for CO and Odor Sensors
    //adc1_config_channel_atten(CO_SENSOR_ADC, ADC_ATTEN_CO);
    //adc1_config_channel_atten(ODOR_SENSOR_ADC, ADC_ATTEN_ODOR);

    /// Init I2C Driver
    initI2CDriver();

    /// Init I2C for AHT20 (Temperature Sensor)
    initTemperatureSensor();

    /// Init I2C for LIS3DH (Accelerometer)
    initAccelerometer();
}

uint32_t readCOSensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &raw);
    return raw;
}

uint32_t readOdorSensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &raw);
    return raw;
}

float readTemperatureSensor() {
    uint8_t writeBuf[3] = {0xAC, 0x33, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, AHT20_ADDR, writeBuf, sizeof(writeBuf), 1000 / portTICK_PERIOD_MS);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t recBuf[6] = {0};
    i2c_master_read_from_device(I2C_NUM_0, AHT20_ADDR, recBuf, sizeof(recBuf), 1000 / portTICK_PERIOD_MS);

    uint32_t tdata = recBuf[3] & 0x0F;
    tdata <<= 8;
    tdata |= recBuf[4];
    tdata <<= 8;
    tdata |= recBuf[5];
    return ((float) tdata * 200 / 0x100000) - 50;
}

Acceleration readAccelerometer() {
    uint8_t data[6];
    Acceleration acceleration;
    uint8_t writeBuf[1] = {LIS3DH_REG_OUT_X_L | 0x80,};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, writeBuf, sizeof(writeBuf), 1000 / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_NUM_0, LIS3DH_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    int16_t x, y, z;
    x = data[0];
    x |= ((uint16_t) data[1]) << 8;
    y = data[2];
    y |= ((uint16_t) data[3]) << 8;
    z = data[4];
    z |= ((uint16_t) data[5]) << 8;
    int lsb_value = 2;
    acceleration.x = lsb_value * ((float) x / 16000);
    acceleration.y = lsb_value * ((float) y / 16000);
    acceleration.z = lsb_value * ((float) z / 16000);

    return acceleration;
}

void calibrateOdor(int numValues, float *mean, float *stdDev) {
    uint32_t sensorValues[numValues];

    for (int i = 0; i < numValues; i++) {
        sensorValues[i] = readOdorSensor();
        *mean += sensorValues[i];
    }
    *mean = *mean / numValues;

    for (int i = 0; i < numValues; i++) {
        *stdDev += pow(sensorValues[i] - *mean, 2);
    }
    *stdDev = sqrt(*stdDev / numValues);
}

void calibrateCO(int numValues, float *mean, float *stdDev) {
    uint32_t sensorValues[numValues];

    for (int i = 0; i < numValues; i++) {
        sensorValues[i] = readCOSensor();
        *mean += sensorValues[i];
    }
    *mean = *mean / numValues;

    for (int i = 0; i < numValues; i++) {
        *stdDev += pow(sensorValues[i] - *mean, 2);
    }
    *stdDev = sqrt(*stdDev / numValues);
}

void calibrateTemperature(int numValues, float *mean, float *stdDev) {
    float sensorValues[numValues];

    for (int i = 0; i < numValues; i++) {
        sensorValues[i] = readTemperatureSensor();
        *mean += sensorValues[i];
    }
    *mean = *mean / numValues;

    for (int i = 0; i < numValues; i++) {
        *stdDev += pow(sensorValues[i] - *mean, 2);
    }
    *stdDev = sqrt(*stdDev / numValues);
}

void calibrateAccelerometer(int numValues, Acceleration *mean, Acceleration *stdDev) {
    Acceleration sensorValues[numValues];

    for (int i = 0; i < numValues; i++) {
        Acceleration temp = readAccelerometer();
        sensorValues[i] = temp;
        mean->x += temp.x;
        mean->y += temp.y;
        mean->z += temp.z;
    }
    mean->x /= numValues;
    mean->y /= numValues;
    mean->z /= numValues;

    for (int i = 0; i < numValues; i++) {
        stdDev->x += pow(sensorValues[i].x - mean->x, 2);
        stdDev->y += pow(sensorValues[i].y - mean->y, 2);
        stdDev->z += pow(sensorValues[i].z - mean->z, 2);
    }
    stdDev->x = sqrt(stdDev->x / numValues);
    stdDev->y = sqrt(stdDev->y / numValues);
    stdDev->z = sqrt(stdDev->z / numValues);
}

