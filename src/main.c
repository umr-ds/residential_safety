#include <stdio.h>
#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "sensor_reader.h"
#include "communication.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "ulp_main.h"
#include "ulp.h"
#include "ulp_adc.h"
#include "math.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *MEASUREMENT = "MEASUREMENT";
static const char *INFO = "INFO";

RTC_DATA_ATTR int calibrated = 0;
RTC_DATA_ATTR int voting = 0;

SemaphoreHandle_t buttonSemaphore;
Message message;

void IRAM_ATTR button_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(buttonSemaphore, NULL);
}

void button_task(void *pvParameter) {
    static int led_level = 0;
    while (1) {
        // Wait for the semaphore to be given by the ISR
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            if(led_level == 0) set_led_level(1);
            else if (led_level == 1) set_led_level(0);
            led_level = led_level^1;
        }
    }
}

static void init_ulp_program(void) {
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_FSM,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

    ulp_wakeup_flag_co = 0;
    ulp_wakeup_flag_odor = 0;
    ulp_high_thr_co = (int)mean_co+3*(int)stdDev_co;
    ulp_high_thr_odor= (int)mean_odor+3*(int)stdDev_odor;
    esp_deep_sleep_disable_rom_logging();
}

static void start_ulp_program(void){
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void calibrate() {
    ESP_LOGI(MEASUREMENT, "Calibrating");
    calibrateTemperature(NUM_CALIBRATION_VALUES, &mean_temperature, &stdDev_temperature);
    calibrateCO(NUM_CALIBRATION_VALUES, &mean_co, &stdDev_co);
    calibrateOdor(NUM_CALIBRATION_VALUES, &mean_odor, &stdDev_odor);
    calibrateAccelerometer(NUM_CALIBRATION_VALUES, &mean_accel, &stdDev_accel);
    calibrated = 1;

    ESP_LOGI(MEASUREMENT, "CO Mean: %f, CO StdDev: %f, "
                          "Odor Mean: %f, Odor StdDev: %f "
                          "Temp Mean: %f, Temp StdDev: %f"
                          "Accel Mean X: %f, Accel Mean Y: %f, Accel Mean Z: %f,"
                          "Accel StdDev X: %f, Accel StdDev Y: %f, Accel StdDev Z: %f\n",
             mean_co, stdDev_co,
             mean_odor, stdDev_odor,
             mean_temperature, stdDev_temperature,
             mean_accel.x, mean_accel.y, mean_accel.z,
             stdDev_accel.x, stdDev_accel.y, stdDev_accel.z);
}

void app_main(void)
{
    ESP_LOGI(INFO, "Initializing Sensors.\n");
    initSensors();
    set_led_level(1);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    set_led_level(0);
    initButton(button_isr_handler);
    buttonSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(&button_task, "button_task", 512, NULL, 5, NULL);

    initWifi();
    initESPNOW(espnow_recv_cb, espnow_send_cb);
    if(calibrated == 0){
        calibrate();
        int64_t start_time = esp_timer_get_time();
        // Wait for 30 seconds
        while((esp_timer_get_time()-start_time <= 1000*1000*30)) {
            if(check_calibration_complete() == 1) break;
            printf("Check calibration complete status: %i\n", check_calibration_complete());
            send_calibration_complete();
            ESP_LOGI(INFO, "Waiting for calibration complete\n");
            vTaskDelay(2000/ portTICK_PERIOD_MS);
        }
    }

    configureInterruptAccelerometer();
    // Deinit ADC to be used for ulp again
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch(cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(INFO, "Wakeup from timer\n");
            voting = false;
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            ESP_LOGI(INFO, "Wakeup from ulp\n");
            ulp_wakeup_flag_odor &= UINT16_MAX;
            ulp_wakeup_flag_co &= UINT16_MAX;
            if(ulp_wakeup_flag_co == 1) ESP_LOGI(INFO, "CO sensor exceeded threshold: %"PRIu32"\n", ulp_high_thr_co);
            if(ulp_wakeup_flag_odor == 1) ESP_LOGI(INFO, "Odor sensor exceeded threshold: %"PRIu32"\n", ulp_high_thr_odor);
            voting = true;
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            if((esp_sleep_get_ext1_wakeup_status() >> LEAKAGE_SENSOR_PIN) & 0x01){
                voting = true;
                ESP_LOGI(INFO, "Wakeup by Leakage Pin\n");
            }
            if((esp_sleep_get_ext1_wakeup_status() >> PIR_SENSOR_PIN) & 0x01){
                ESP_LOGI(INFO, "Wakeup by PIR Pin\n");
                voting = true;
            }
            if((esp_sleep_get_ext1_wakeup_status() >> HALL_SENSOR_PIN) & 0x01){
                ESP_LOGI(INFO, "Wakeup by Hall Pin\n");
                voting = true;
            }
            if((esp_sleep_get_ext1_wakeup_status() >> ACCELEROMETER_INTR_PIN) & 0x01) {
                ESP_LOGI(INFO, "Wakeup by Acclerometer Pin\n");
                voting = true;
            }
            break;
        default:
            init_ulp_program();

            break;
    }
    if(voting == false) {
        ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup((1ULL << HALL_SENSOR_PIN) | ( 1ULL << PIR_SENSOR_PIN) | (1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN), ESP_EXT1_WAKEUP_ANY_HIGH));
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(1000*1000*60));
        rtc_gpio_isolate(GPIO_NUM_12);
        rtc_gpio_isolate(GPIO_NUM_15);
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(HALL_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(LEAKAGE_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ACCELEROMETER_INTR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(HALL_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(LEAKAGE_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ACCELEROMETER_INTR_PIN));
        start_ulp_program();
        esp_deep_sleep_start();
    }

}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    message = *(Message*)data;

    printf("Message received from: %i\n", message.node_id);
    printf("Message type: %i\n",message.message_type);
    switch(message.message_type){
        case CALIBRATION_MESSAGE_TYPE:
            set_calibration_finished(recv_info->src_addr);
            break;
        default:
            break;
    }


}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

}


