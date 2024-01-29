#include <stdio.h>
#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "ulp_main.h"
#include "ulp.h"
#include "ulp_adc.h"
#include "math.h"
#include "communication.h"
#include "voting.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


RTC_SLOW_ATTR  volatile int alarm_mode = 0;
RTC_SLOW_ATTR  int calibrated = 0;
RTC_SLOW_ATTR  int voting_started = 0;

void IRAM_ATTR button_isr_handler(void *arg) {
    alarm_mode = alarm_mode^1;
    if(alarm_mode == 0) set_led_level(0);
    else if(alarm_mode == 1) set_led_level(1);
}

void voting_task(void *pvParameter) {
    printf("Voting task started\n");
    int event_flag = (int)pvParameter;
    init_votes();
    Message msg = start_voting(event_flag);
    set_vote(get_node_id(get_own_mac()), msg.data.voting_msg.vote);
    set_sensor_voted(get_node_id(get_own_mac()));
    while(1){
        broadcast_message(msg);
        if(check_votes() == 1){
            printf("All votes finished\n");
            printf("Result: %i\n", calculate_decision(event_flag));
            voting_started=0;
            vTaskDelete(NULL);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
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
    ulp_high_thr_co = (int) mean_co + 3 * (int) stdDev_co;
    ulp_high_thr_odor = (int) mean_odor + 3 * (int) stdDev_odor;
    esp_deep_sleep_disable_rom_logging();
}

static void start_ulp_program(void) {
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void calibrate() {
    printf("Calibrating\n");
    calibrateCO(NUM_CALIBRATION_VALUES, &mean_co, &stdDev_co);
    calibrateOdor(NUM_CALIBRATION_VALUES, &mean_odor, &stdDev_odor);
    calibrateAccelerometer(NUM_CALIBRATION_VALUES, &mean_accel, &stdDev_accel);

    printf("CO Mean: %f, CO StdDev: %f, "
                          "Odor Mean: %f, Odor StdDev: %f "
                          "Accel Mean X: %f, Accel Mean Y: %f, Accel Mean Z: %f,"
                          "Accel StdDev X: %f, Accel StdDev Y: %f, Accel StdDev Z: %f\n",
             mean_co, stdDev_co,
             mean_odor, stdDev_odor,
             mean_accel.x, mean_accel.y, mean_accel.z,
             stdDev_accel.x, stdDev_accel.y, stdDev_accel.z);
    calibrated = 1;
}

void app_main(void) {
    printf("Starting\n");
    initLED();
    initSensors();
    set_led_level(1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_led_level(0);
    initButton(button_isr_handler);

    initWifi();
    initESPNOW(espnow_recv_cb, espnow_send_cb);
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if(calibrated == 0) {
        calibrate();
    }
    configureInterruptAccelerometer();
    uint64_t start_timer = esp_timer_get_time();
    // wait for possible messages
    while(esp_timer_get_time()-start_timer <= 10*1000*1000){
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
   // Deinit ADC to be used for ulp again
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

   switch (cause) {
       case ESP_SLEEP_WAKEUP_TIMER:
           printf("Wakeup from timer\n");
           voting_started = 0;
           break;
       case ESP_SLEEP_WAKEUP_ULP:
           printf("Wakeup from ulp\n");
           ulp_wakeup_flag_odor &= UINT16_MAX;
           ulp_wakeup_flag_co &= UINT16_MAX;
           if (ulp_wakeup_flag_co == 1){
               printf("CO sensor exceeded threshold: %"PRIu32"\n", ulp_high_thr_co);
               xTaskCreate(&voting_task, "voting_task", 2048, (void *) FIRE_FLAG, 5, NULL);
           } else if (ulp_wakeup_flag_odor == 1){
               printf("Odor sensor exceeded threshold: %"PRIu32"\n", ulp_high_thr_odor);
               xTaskCreate(&voting_task, "voting_task", 2048, (void *) GAS_LEAKAGE_FLAG, 5, NULL);
           }
           voting_started = 1;
           break;
       case ESP_SLEEP_WAKEUP_EXT1:
           if ((esp_sleep_get_ext1_wakeup_status() >> LEAKAGE_SENSOR_PIN) & 0x01) {
               printf("Wakeup by Leakage Pin\n");
               xTaskCreate(&voting_task, "voting_task", 2048, (void *) WATER_LEAKAGE_FLAG, 5, NULL);
               voting_started = 1;
           }
           if ((esp_sleep_get_ext1_wakeup_status() >> HALL_SENSOR_PIN) & 0x01) {
               printf("Wakeup by Hall Pin\n");
               xTaskCreate(&voting_task, "voting_task", 2048, (void *) INTRUSION_FLAG, 5, NULL);
               voting_started = 1;
           }
           if ((esp_sleep_get_ext1_wakeup_status() >> ACCELEROMETER_INTR_PIN) & 0x01) {
               printf("Wakeup by Acclerometer Pin\n");
               xTaskCreate(&voting_task, "voting_task", 2048, (void *) SHOCK_FLAG, 5, NULL);
               voting_started = 1;
           }
           break;
       default:
           init_ulp_program();
           voting_started = 0;
           break;
   }
   if (voting_started == 0) {
       printf("Starting deepsleep\n");
       ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
       //ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(
       //       (1ULL << HALL_SENSOR_PIN) | (1ULL << LEAKAGE_SENSOR_PIN) |
       //       (1ULL << ACCELEROMETER_INTR_PIN), ESP_EXT1_WAKEUP_ANY_HIGH));
       ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(
               (1ULL << HALL_SENSOR_PIN) | (1ULL << LEAKAGE_SENSOR_PIN), ESP_EXT1_WAKEUP_ANY_HIGH));
       ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(1000 * 1000 * 20));
       rtc_gpio_isolate(GPIO_NUM_12);
       rtc_gpio_isolate(GPIO_NUM_15);
       ESP_ERROR_CHECK(rtc_gpio_pullup_dis(HALL_SENSOR_PIN));
       ESP_ERROR_CHECK(rtc_gpio_pullup_dis(LEAKAGE_SENSOR_PIN));
       //ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ACCELEROMETER_INTR_PIN));
       ESP_ERROR_CHECK(rtc_gpio_pulldown_en(HALL_SENSOR_PIN));
       ESP_ERROR_CHECK(rtc_gpio_pulldown_en(LEAKAGE_SENSOR_PIN));
       //ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ACCELEROMETER_INTR_PIN));
       start_ulp_program();
       esp_deep_sleep_start();
   } else {
       printf("Inside else\n");
       while(voting_started == 1) {
           printf( "Inside while, waiting for voting\n");
           vTaskDelay(100/portTICK_PERIOD_MS);
       }
   }
}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    Message message = *(Message*)data;
    switch(message.message_type){
        case VOTING_MESSAGE_TYPE:
            printf("voting message received with event: %u, and vote: %f :\n", message.data.voting_msg.event_flag, message.data.voting_msg.vote);
            if(voting_started == 0){
                //Sensor has not recognized anything, will start because of messages
                voting_started = 1;
                xTaskCreate(&voting_task, "voting_task", 2048, (void *) (intptr_t )message.data.voting_msg.event_flag, 5, NULL);
            } else {
                // Sensor has already started, will calculate vote
                printf("inside else\n");
                set_vote(message.src_node_id, message.data.voting_msg.vote);
                set_sensor_voted(message.src_node_id);
            }

            break;
        case ACK_MESSAGE_TYPE:
            printf("Received acknowledgement\n");
        default:
            break;
    }
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

}





