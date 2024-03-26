#include "sensor_reader.h"
#include "driver/i2c.h"
#include "math.h"

adc_oneshot_unit_handle_t adc1_handle;
static volatile bool button_pressed = false;

RTC_DATA_ATTR uint32_t mean_odor = 0;
RTC_DATA_ATTR uint32_t mean_co = 0;
RTC_DATA_ATTR uint32_t mean_temp = 0;


// ISR for the user button. Sets the flag 'button_pressed' to tur once the button generated an interrupt
void IRAM_ATTR isr_handler(void *arg) {
    gpio_num_t pin = (gpio_num_t) arg;
    switch (pin) {
        case BUTTON_PIN:
            button_pressed = true;
            break;
        default:
            break;
    }
}

// Resets the 'button_pressed' flag to false
void reset_button_pressed() {
    button_pressed = false;
}

// Returns if th user button was pressed
bool was_button_pressed() {
    return button_pressed;
}

// Initializes the button in interrupt mode and adds the ISR handler
void init_button() {
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << BUTTON_PIN),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pull_up_en = GPIO_PULLUP_ENABLE, // since the button is low active
            .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, isr_handler, (void *) BUTTON_PIN);
}

// Initializes the LED to indicate if the alarm mode is active
void init_led() {
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

// Init ADCs for CO and odor Sensors
void init_adc_channels() {

    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_2_5,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

}

// Init I2C for accelerometer and temperature sensor
void init_i2c_driver() {
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

// Init the AHT20 temperature sensor according to the datasheet
void init_temperature_sensor() {
    uint8_t writeBuf[3] = {0xBE, 0x08, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, AHT20_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               100 / portTICK_PERIOD_MS);
}

// Read the raw values of the temperature sensor according to the datasheet
// Converts the raw value to °C
// Returns temperature in °C
float read_temperature_sensor() {
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

// Returns the content of the register with address 'reg' of the accelerometer
uint8_t lis3dh_read_register(uint8_t reg) {
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

// Initializes the LIS3DH accelerometer
void lis3dh_init(uint8_t threshold) {
    // Data rate: 100Hz, Low-Power disabled, X,Y,Z Axes enabled
    uint8_t writeBuf[2] = {LIS3DH_REG_CTRL1, 0x57};
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL2;
    writeBuf[1] = 0x09; // High pass filter normal mode (reset by reading REF register), enabled for INT1
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL3;
    writeBuf[1] = 0x40; //  INT1 enabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL4;
    writeBuf[1] = 0x00; // Continuous update, Little Endian, 2G-Scale, high-resoultion disabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_CTRL5;
    writeBuf[1] = 0x08; // Latch interrupt enabled
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_INT1_THS;
    writeBuf[1] = threshold; // 16mg for 2g scale
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    writeBuf[0] = LIS3DH_REG_INT1_DUR;
    writeBuf[1] = 0x01;
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

    lis3dh_read_register(LIS3DH_REG_REFERENCE);

    writeBuf[0] = LIS3DH_REG_INT1_CFG;
    writeBuf[1] = 0x2A;
    i2c_master_write_to_device(I2C_NUM_0, LIS3DH_ADDR, (const uint8_t *) writeBuf, sizeof(writeBuf),
                               1000 / portTICK_PERIOD_MS);

}

// Read the INT1_SRC register to reset the interrupt pin for new events
void lis3dh_reset_interrupt() {
    lis3dh_read_register(LIS3DH_REG_INT1_SRC);
}

// Read the raw values of the register for all three axes
// Converts the raw values to an acceleration in g (m/s²) and returns it
Acceleration read_accelerometer() {
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
    float sensitivity = 0.001;
    acceleration.x = (float) x * sensitivity;
    acceleration.y = (float) y * sensitivity;
    acceleration.z = (float) z * sensitivity;

    return acceleration;
}

// Initialize the water leakage sensor in input mode
void init_leakage_sensor() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LEAKAGE_SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

// Returns 1 if the water probe has detected a liquid, 0 otherwise
int read_leakage_sensor() {
    return gpio_get_level(LEAKAGE_SENSOR_PIN);
}

// Initialize the PIR sensor in input mode
void init_pir_sensor() {
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << PIR_SENSOR_PIN),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);
}

// Returns 1 if the PIR sensor has detected movement, 0 otherwise
int read_pir_sensor() {
    return gpio_get_level(PIR_SENSOR_PIN);
}

// Initialize the hall sensor in input mode
void init_hall_sensor() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << HALL_SENSOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

// Returns 1 if the hall sensor has detected a magnetic field (open door), 0 otherwise
int read_hall_sensor() {
    return gpio_get_level(HALL_SENSOR_PIN);
}

// Returns the voltage output by the co sensor
uint32_t read_co_sensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, CO_SENSOR_ADC_CHANNEL, &raw);
    return raw;
}

// Returns the voltage output by the odor sensor
uint32_t read_odor_sensor() {
    int raw = 0;
    adc_oneshot_read(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &raw);
    return raw;
}

// Calculates a mean value of the odor sensor for calibration with 'numValues' measurements
void calculate_odor_mean(int numValues) {
    uint32_t sensorValues[numValues];
    uint32_t mean = 0;
    for (int i = 0; i < numValues; i++) {
        sensorValues[i] = read_odor_sensor();
        mean += sensorValues[i];
    }
    mean_odor = (uint32_t) (mean / numValues);
}

// Calculates a mean value of the co sensor for calibration with 'numValues' measurements
void calculate_co_mean(int numValues) {
    uint32_t sensorValues[numValues];
    uint32_t mean = 0;
    for (int i = 0; i < numValues; i++) {
        sensorValues[i] = read_co_sensor();
        mean += sensorValues[i];
    }
    mean_co = (uint32_t) (mean / numValues);
}

// Returns the calculated co mean value
uint32_t get_co_mean() {
    return mean_co;
}

// Returns the calculated odor mean value
uint32_t get_odor_mean() {
    return mean_odor;
}

// Calculates a mean value of the co sensor for calibration with 'numValues' measurements
void calculate_temperature_mean(int numValues) {
    float values[numValues];
    float mean = 0.0;
    for (int i = 0; i < numValues; i++) {
        float temp = read_temperature_sensor();
        values[i] = temp;
        mean += temp;
    }
    mean_temp = mean / numValues;
}

// Returns the calculated temperature mean value
float get_temperature_mean() {
    return mean_temp;
}