#include "sensor_reader.h"
#include "driver/i2c.h"
#include "math.h"

adc_oneshot_unit_handle_t adc1_handle;

void initButton(gpio_isr_t button_isr) {
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << BUTTON_PIN),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, (void *) BUTTON_PIN);
}

void initLED() {
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << LED_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);
}

void set_led_level(uint8_t level) {
    gpio_set_level(LED_PIN, level);
}

int get_led_level() {
    return gpio_get_level(LED_PIN);
}

void initADCs(){
    /// Init ADCs for CO and Odor Sensors
    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_0,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

}

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

void initTemperatureSensor() {
    uint8_t writeBuf[3] = {0xBE, 0x08, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, AHT20_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
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
uint8_t lis3dh_read_register(uint8_t reg){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS3DH_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(1);  // Add a small delay to ensure data is ready

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS3DH_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t value;
    i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return value;
}

void lis3dh_init() {
    // Data rate: 100Hz, Low-Power disabled, X,Y,Z Axes enabled
    uint8_t writeBuf[2] = {LIS3DH_REG_CTRL1, 0x57};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);


    writeBuf[0] = LIS3DH_REG_CTRL4;
    writeBuf[1] = 0x18; // Continuous update, Little Endian, 4G-Scale, high-resoultion enabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
}

void lis3dh_config_interrupt(uint8_t threshold){

    uint8_t writeBuf[2];
    writeBuf[0] = LIS3DH_REG_CTRL2;
    writeBuf[1] = 0x09; // High pass filter normal mode (reset by reading REF register), enabled for INT1
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL3;
    writeBuf[1] = 0x40; //  INT1 enabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL5;
    writeBuf[1] = 0x08; // Latch interrupt enabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_INT1_THS;
    writeBuf[1] = threshold; // 1LSB at 4G-Scale = 32mg
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_INT1_DUR;
    writeBuf[1] = 0x00;
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_INT1_CFG;
    writeBuf[1] = 0x2A;
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);
}

void lis3dh_reset_interrupt(){
    lis3dh_read_register(0x21); //read register to reset high-pass filter
    lis3dh_read_register(0x26); //read register to set reference acceleration
    lis3dh_read_register(0x31);
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
    x = x >> 4;
    y = ((int16_t) data[3] << 8) + (uint16_t) data[2];
    y = y >> 4;
    z = ((int16_t) data[5] << 8) + (uint16_t) data[4];
    z = z >> 4;
    float sensitivity = 0.002; // 2mg per digit
    acceleration.x = (float) x * sensitivity;
    acceleration.y = (float) y * sensitivity;
    acceleration.z = (float) z * sensitivity;

    return acceleration;
}

void initLeakageSensor() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LEAKAGE_SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

int readLeakageSensor() {
    return gpio_get_level(LEAKAGE_SENSOR_PIN);
}

void initPIRSensor(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIR_SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

int readPIRSensor() {
    return gpio_get_level(PIR_SENSOR_PIN);
}

void initHallSensor(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << HALL_SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

int readHallSensor() {
    return gpio_get_level(HALL_SENSOR_PIN);
}

uint32_t readCOSensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, CO_SENSOR_ADC_CHANNEL, &raw);
    return raw;
}

uint32_t readOdorSensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &raw);
    return raw;
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

