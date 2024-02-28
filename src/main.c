#include "communication.h"
#include "driver/gpio.h"
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "main.h"
#include "ulp_main.h"
#include "ulp.h"
#include "ulp_adc.h"
#include "voting.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


static volatile bool voting_started = false;

RTC_DATA_ATTR static volatile bool alarm_mode = false;
RTC_DATA_ATTR static bool calibrated = false;
RTC_DATA_ATTR TaskHandle_t taskHandle = NULL;
RTC_DATA_ATTR static uint8_t calibration_reset_counter = 0;

static void init_ulp_program(void) {

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
    adc_oneshot_del_unit(adc1_handle);
    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_FSM,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_2_5,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    config.atten = ADC_ATTEN_DB_6;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

    ulp_wakeup_flag_co = 0;
    ulp_wakeup_flag_odor = 0;
    ulp_wakeup_flag_button = 0;
    ulp_high_thr_co = get_co_mean() - 200 >= 3895 ? 4095 : get_co_mean()  + 200;
    ulp_high_thr_odor = get_odor_mean() - 200 >= 3895 ? 4095 : get_odor_mean() + 200;
    printf("Ulp odor threshold: %lu\n", ulp_high_thr_odor);
    esp_deep_sleep_disable_rom_logging();
}

static void start_ulp_program(void) {
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}


void voting_task(void *pvParameter) {
    int event_flag = (int) pvParameter;
    init_votes();
    Message msg = {
            .message_type = VOTING_REQUEST_MESSAGE_TYPE,
            .event_flag = event_flag,
    };
    //if(param.event_flag == INTRUSION_FLAG)
    set_vote(get_node_id(get_own_mac()), calculate_vote(event_flag));
    set_sensor_voted(get_node_id(get_own_mac()));
    set_node_weight(get_node_id(get_own_mac()), get_own_node_weight(event_flag));
    uint64_t start_time = esp_timer_get_time();


    while ((esp_timer_get_time() - start_time) <= VOTING_TIMEOUT_INTERVAL) {
        if (check_all_nodes_voted() == true) break;
        for (int i = 0; i < MAX_NUM_SENSORS; i++) {
            // Skip if we are the current node
            if (get_node_id(get_own_mac()) == i) continue;
            // Skip if node has already voted
            if (get_sensor_voted(i) == true) continue;
            send_message_to_node(msg, i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    float neccessary_majority, vote = 0;
    bool decision = calculate_decision(event_flag, &vote, &neccessary_majority);
    msg.message_type = VOTING_RESULT_MESSAGE_TYPE;
    msg.event_flag = event_flag;
    msg.data.voting_result_msg.decision = decision;
    msg.data.voting_result_msg.vote = vote;
    msg.data.voting_result_msg.necessary_majority = neccessary_majority;
    if (decision) {
        printf("Nodes are accepting event :%i with Vote: %f and neccessary majority was: %f\n",
               msg.event_flag,
               msg.data.voting_result_msg.vote,
               msg.data.voting_result_msg.necessary_majority);
    } else {
        printf("Nodes are declining event :%i with Vote: %f and neccessary majority was: %f\n",
               msg.event_flag,
               msg.data.voting_result_msg.vote,
               msg.data.voting_result_msg.necessary_majority);
    }

    voting_started = false;
    taskHandle = NULL;
    vTaskDelete(taskHandle);
}

void button_task(){
    while(1){
        if(wasButtonPressed()){
            printf("button was pressed\n");
            alarm_mode = !alarm_mode;
            vTaskDelay(1000/portTICK_PERIOD_MS);
            resetButtonPressed();
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}



void app_main() {
    initLED();
    initButton();
    xTaskCreatePinnedToCore(&button_task, "buttonTask", 2048, NULL, 5, NULL, 1);
    if(alarm_mode) set_led_level(1);
    initADCs();
    initI2CDriver();
    initTemperatureSensor();

    if (calibration_reset_counter == 10) {
        calculate_temperature_mean(10);
        //calculate_co_mean(NUM_CALIBRATION_VALUES);
        //calculate_odor_mean(NUM_CALIBRATION_VALUES);
        calibration_reset_counter = 0;
    }
    if (calibrated == false) {
        printf("Calibrating to preheat gas sensors. This may take 2 minutes, and is only done once the node was ejected from power\n");
        uint64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) <= 10 * MICROSEC_TO_SEC) {
            readOdorSensor();
            readCOSensor();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        calculate_co_mean(NUM_CALIBRATION_VALUES);
        calculate_odor_mean(NUM_CALIBRATION_VALUES);
        printf("Odor mean: %lu, CO mean: %lu\n", get_odor_mean(), get_co_mean());
        calculate_temperature_mean(10);
        lis3dh_init(0x3F);
        calibrated = true;
    }

    initWifi();
    initESPNOW(espnow_recv_cb);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ulp_last_result_odor &= UINT16_MAX;
            ulp_last_result_co &= UINT16_MAX;
            printf("Wakeup by timer, last odor value: %lu, last co value: %lu\n", ulp_last_result_odor, ulp_last_result_co);
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            if ((esp_sleep_get_ext1_wakeup_status() >> ACCELEROMETER_INTR_PIN) & 0x01) {
                printf("Wakeup from Accelerometer Interrupt Pin\n");
                if (taskHandle == NULL) {
                    voting_started = true;
                    xTaskCreate(&voting_task, "voting_task", 2048, (void *) SHOCK_FLAG, 5, &taskHandle);
                }
            } else if ((esp_sleep_get_ext1_wakeup_status() >> PIR_SENSOR_PIN) & 0x01) {
                    printf("Wakeup from PIR Sensor");
                if (taskHandle == NULL) {
                    voting_started = true;
                    xTaskCreate(&voting_task, "voting_task", 4096, (void *) INTRUSION_FLAG, 5, &taskHandle);
                }

            } else if ((esp_sleep_get_ext1_wakeup_status() >> LEAKAGE_SENSOR_PIN) & 0x01) {
                printf("Wakeup from Leakage Pin\n");
                if (taskHandle == NULL) {
                    voting_started = true;
                    xTaskCreate(&voting_task, "voting_task", 4096, (void *) WATER_LEAKAGE_FLAG, 5, &taskHandle);
                }
            }
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            printf("ULP wakeup\n");
            ulp_wakeup_flag_odor &= UINT16_MAX;
            ulp_wakeup_flag_co &= UINT16_MAX;
            ulp_wakeup_flag_button &= UINT16_MAX;
            if (ulp_wakeup_flag_button == 1) {
                alarm_mode = !alarm_mode;
                set_led_level(1);
                printf("Button wakeup, alarm mode is:%u\n", alarm_mode);
            } else if (ulp_wakeup_flag_odor == 1 || ulp_wakeup_flag_co == 1) {
                if (taskHandle == NULL) {
                    voting_started = true;
                    xTaskCreate(&voting_task, "voting_task", 2048, (void *) FIRE_OR_GAS_FLAG, 5, &taskHandle);
                }
            }
            break;
        default:
            break;
    }

    if (voting_started == false) {
        uint64_t start_time = esp_timer_get_time();
        while (esp_timer_get_time() - start_time <= MESSAGE_WAITING_INTERVAL) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    while (voting_started == true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(rtc_gpio_init(LEAKAGE_SENSOR_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(LEAKAGE_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(LEAKAGE_SENSOR_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(LEAKAGE_SENSOR_PIN));

    ESP_ERROR_CHECK(rtc_gpio_init(ACCELEROMETER_INTR_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(ACCELEROMETER_INTR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ACCELEROMETER_INTR_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ACCELEROMETER_INTR_PIN));

    gpio_intr_disable(BUTTON_PIN);
    ESP_ERROR_CHECK(rtc_gpio_init(BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(BUTTON_PIN));

    if (alarm_mode) {
        printf("Alarm mode enabled\n");
        ESP_ERROR_CHECK(rtc_gpio_init(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(PIR_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(PIR_SENSOR_PIN));
        esp_sleep_enable_ext1_wakeup(
                ((1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN) | (1ULL << PIR_SENSOR_PIN) ), ESP_EXT1_WAKEUP_ANY_HIGH);
    } else {
        esp_sleep_enable_ext1_wakeup(((1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN)),
                                     ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    printf("Going to sleep mode\n");
    calibration_reset_counter++;
    init_ulp_program();
    esp_sleep_enable_ulp_wakeup();
    start_ulp_program();
    esp_deep_sleep(DEEPSLEEP_INTERVAL);
}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    Message received_message = *(Message *) data;
    switch (received_message.message_type) {
        case VOTING_REQUEST_MESSAGE_TYPE:
            if (taskHandle == NULL) {
                int event_flag = received_message.event_flag;
                // do not create another task, if one is already running
                voting_started = true;
                printf("Task not present yet, creating it with event flag: %i\n", event_flag);
                xTaskCreate(&voting_task, "voting_task", 4096, (void *) event_flag,
                            5, &taskHandle);
            }
            while (get_sensor_voted(get_node_id(get_own_mac())) == false) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            Message voting_msg = {
                    .message_type = VOTING_ANSWER_MESSAGE_TYPE,
                    .event_flag = received_message.event_flag,
                    .data.voting_answer_msg.vote = get_vote(get_node_id(get_own_mac())),
                    .data.voting_answer_msg.vote_weight = get_own_node_weight(
                            received_message.event_flag),
            };
            send_message_to_node(voting_msg, get_node_id(recv_info->src_addr));
            break;
        case VOTING_ANSWER_MESSAGE_TYPE:
            set_node_weight(get_node_id(recv_info->src_addr), received_message.data.voting_answer_msg.vote_weight);
            set_vote(get_node_id(recv_info->src_addr), received_message.data.voting_answer_msg.vote);
            set_sensor_voted(get_node_id(recv_info->src_addr));
            break;
        case VOTING_RESULT_MESSAGE_TYPE:
            printf(received_message.data.voting_result_msg.decision == true ?
                   "Nodes are accepting event :%i with Vote: %f and neccessary majority was: %f\n"
                                                                            : "Nodes are not declining event :%i with Vote: %f and neccessary majority was: %f\n",
                   received_message.event_flag,
                   received_message.data.voting_result_msg.vote,
                   received_message.data.voting_result_msg.necessary_majority);
            break;
        default:
            break;
    }
}