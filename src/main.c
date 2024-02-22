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


static bool task_ready_to_delete = false;
RTC_SLOW_ATTR volatile bool voting_started = false;
RTC_SLOW_ATTR volatile bool alarm_mode = false;
RTC_SLOW_ATTR bool calibrated = false;
TaskHandle_t taskHandle = NULL;

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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

    ulp_wakeup_flag_co = 0;
    ulp_wakeup_flag_odor = 0;
    ulp_wakeup_flag_button = 0;
    ulp_high_thr_co = (uint32_t) mean_co + 150;
    ulp_high_thr_odor = (uint32_t) mean_odor + 50;
    //printf("CO high threshold is %lu\n", ulp_high_thr_co);
    //printf("Odor high threshold is %lu\n", ulp_high_thr_odor);
    esp_deep_sleep_disable_rom_logging();
}

static void start_ulp_program(void) {
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void delete_task(TaskHandle_t task_handle) {
    vTaskDelete(task_handle);
    voting_started = false;
    task_ready_to_delete = false;
}

void voting_task(void *pvParameter) {
    event param = *(event *) pvParameter;
    //printf("Voting task started for event flag: %u\n", param.event_flag);
    init_votes();
    Message msg = {
            .message_type = VOTING_REQUEST_MESSAGE_TYPE,
            .data.voting_request_msg.event = param,
    };
    set_vote(get_node_id(get_own_mac()), calculate_vote(param));
    set_sensor_voted(get_node_id(get_own_mac()));
    set_node_weight(get_node_id(get_own_mac()), get_own_node_weight(param.event_flag));
    uint64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) <= VOTING_TIMEOUT_INTERVAL) {
        if (check_all_nodes_voted() == true) break;
        for (int i = 0; i < MAX_NUM_SENSORS; i++) {
            // Skip if we are the current node
            if (get_node_id(get_own_mac()) == i) continue;
            // Skip if node has  already voted
            if (get_sensor_voted(i) == true) continue;
            send_message_to_node(msg, i);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    //printf("Voting finished, calculating decision\n");
    float neccessary_majority, vote = 0;
    bool decision = calculate_decision(param.event_flag, &vote, &neccessary_majority);
    msg.message_type = VOTING_RESULT_MESSAGE_TYPE;
    msg.data.voting_result_msg.event_flag = param.event_flag;
    msg.data.voting_result_msg.decision = decision;
    msg.data.voting_result_msg.vote = vote;
    msg.data.voting_result_msg.necessary_majority = neccessary_majority;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if (get_sensor_voted(i) == true) send_message_to_node(msg, i);
    }
    if (decision) {
        printf("Nodes are accepting event :%i with Vote: %f and neccessary majority was: %f\n",
               msg.data.voting_result_msg.event_flag,
               msg.data.voting_result_msg.vote,
               msg.data.voting_result_msg.necessary_majority);
    } else {
        printf("Nodes are declining event :%i with Vote: %f and neccessary majority was: %f\n",
               msg.data.voting_result_msg.event_flag,
               msg.data.voting_result_msg.vote,
               msg.data.voting_result_msg.necessary_majority);
    }
    task_ready_to_delete = true;
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR button_isr_handler(void *arg) {
    alarm_mode = !alarm_mode;
}


void app_main() {

    initADCs();
    initI2CDriver();
    initButton(button_isr_handler);
    initLED();
    initTemperatureSensor();
    if (calibrated == false) {
        printf("Calibrating to preheat gas sensors. This may take 2 minutes, and is only done once the node was ejected from power\n");
        uint64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) <= CALIBRATION_INTERVAL) {
            readOdorSensor();
            readCOSensor();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        mean_odor = calculate_odor_mean(NUM_CALIBRATION_VALUES);
        mean_co = calculate_co_mean(NUM_CALIBRATION_VALUES);
        current_temp = readTemperatureSensor();
        lis3dh_init();
        lis3dh_config_interrupt(0x13);
        calibrated = true;
    }

    initWifi();
    initESPNOW(espnow_recv_cb, espnow_send_cb);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            printf("Wakeup by timer\n");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            if ((esp_sleep_get_ext1_wakeup_status() >> ACCELEROMETER_INTR_PIN) & 0x01) {
                printf("Wakeup from Accelerometer Interrupt Pin\n");
                if (taskHandle == NULL) {
                    voting_started = true;
                    event event = {
                            .event_flag = SHOCK_FLAG,
                    };
                    xTaskCreate(&voting_task, "voting_task", 2048, (void *) &event, 5, &taskHandle);
                }
            }
            if (((esp_sleep_get_ext1_wakeup_status() >> PIR_SENSOR_PIN) & 0x01) ||
                (esp_sleep_get_ext1_wakeup_status() >> HALL_SENSOR_PIN) & 0x01) {
                if ((esp_sleep_get_ext1_wakeup_status() >> PIR_SENSOR_PIN) & 0x01) printf("Wakeup from PIR Sensor");
                if ((esp_sleep_get_ext1_wakeup_status() >> HALL_SENSOR_PIN) & 0x01) printf("Wakeup from Hall Sensor");
            }
            if ((esp_sleep_get_ext1_wakeup_status() >> LEAKAGE_SENSOR_PIN) & 0x01) {
                printf("Wakeup from Leakage Pin\n");
                if (taskHandle == NULL) {
                    voting_started = true;
                    event event = {
                            .event_flag = WATER_LEAKAGE_FLAG,
                    };
                    xTaskCreate(&voting_task, "voting_task", 2048, (void *) &event, 5, &taskHandle);
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
                printf("Button wakeup, alarm mode is:%u\n", alarm_mode);
            } else if (ulp_wakeup_flag_odor == 1 || ulp_wakeup_flag_co == 1) {
                if (taskHandle == NULL) {
                    voting_started = true;
                    event event = {
                            .event_flag = FIRE_OR_GAS_FLAG,
                            .co_or_odor_event_flag = {
                                    .co_flag = ulp_wakeup_flag_co == 1 ? true : false,
                                    .odor_flag = ulp_wakeup_flag_odor == 1 ? true : false,
                            }
                    };
                    xTaskCreate(&voting_task, "voting_task", 2048, (void *) &event, 5, &taskHandle);

                }
            }
        default:
            break;
    }
    if (voting_started == false) {
        uint64_t start_time = esp_timer_get_time();
        set_led_level(1);
        printf("Waiting for messages\n");
        while (esp_timer_get_time() - start_time <= MESSAGE_WAITING_INTERVAL) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        set_led_level(0);
    }
    while (voting_started == true) {

        if (task_ready_to_delete) delete_task(taskHandle);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    printf("Waiting for messages finished\n");
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
        ESP_ERROR_CHECK(rtc_gpio_init(HALL_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(HALL_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
        ESP_ERROR_CHECK(rtc_gpio_pullup_en(HALL_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(HALL_SENSOR_PIN));

        ESP_ERROR_CHECK(rtc_gpio_init(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(PIR_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
        ESP_ERROR_CHECK(rtc_gpio_pullup_dis(PIR_SENSOR_PIN));
        ESP_ERROR_CHECK(rtc_gpio_pulldown_en(PIR_SENSOR_PIN));
        esp_sleep_enable_ext1_wakeup(
                ((1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN) | (1ULL << HALL_SENSOR_PIN) |
                 (1ULL << PIR_SENSOR_PIN)), ESP_EXT1_WAKEUP_ANY_HIGH);
    } else {
        esp_sleep_enable_ext1_wakeup(((1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN)),
                                     ESP_EXT1_WAKEUP_ANY_HIGH);
    }


    printf("Going to sleep mode\n");
    init_ulp_program();
    esp_sleep_enable_ulp_wakeup();
    start_ulp_program();
    esp_deep_sleep(DEEPSLEEP_INTERVAL);

}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    Message received_message = *(Message *) data;
    switch (received_message.message_type) {
        case ALARM_MODE_MESSAGE_TYPE:
            //printf("button was pressed\n");
            break;
        case VOTING_REQUEST_MESSAGE_TYPE:
            if (taskHandle == NULL) {
                // do not create another task, if one is already running
                printf("Task not present yet, creating it\n");
                voting_started = true;
                event event = {
                        .event_flag = FIRE_OR_GAS_FLAG,
                        .co_or_odor_event_flag = {
                                .co_flag = ulp_wakeup_flag_co == 1 ? true : false,
                                .odor_flag = ulp_wakeup_flag_odor == 1 ? true : false,
                        }
                };
                xTaskCreate(&voting_task, "voting_task", 2048, (void *) &event, 5, &taskHandle);
            }
            while (get_sensor_voted(get_node_id(get_own_mac())) == false) {
                printf("Waiting for node to finish vote\n");
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            //printf("Should send vote %f\n", get_vote(get_node_id(get_own_mac())));
            Message voting_msg = {
                    .message_type = VOTING_ANSWER_MESSAGE_TYPE,
                    .data.voting_answer_msg.event_flag = received_message.data.voting_request_msg.event.event_flag,
                    .data.voting_answer_msg.vote = get_vote(get_node_id(get_own_mac())),
                    .data.voting_answer_msg.vote_weight = get_own_node_weight(
                            received_message.data.voting_request_msg.event.event_flag),
            };
            //printf("Sending vote: %f\n", voting_msg.data.voting_answer_msg.vote);
            send_message_to_node(voting_msg, get_node_id(recv_info->src_addr));
            break;
        case VOTING_ANSWER_MESSAGE_TYPE:
            printf("Received Voting from Node %i with vote: %f\n", get_node_id(recv_info->src_addr),
                   received_message.data.voting_answer_msg.vote);
            set_node_weight(get_node_id(recv_info->src_addr), received_message.data.voting_answer_msg.vote_weight);
            set_vote(get_node_id(recv_info->src_addr), received_message.data.voting_answer_msg.vote);
            set_sensor_voted(get_node_id(recv_info->src_addr));
            break;
        case VOTING_RESULT_MESSAGE_TYPE:
            printf(received_message.data.voting_result_msg.decision == true ?
                   "Nodes are accepting event :%i with Vote: %f and neccessary majority was: %f\n"
                                                                            : "Nodes are not accepting event :%i with Vote: %f and neccessary majority was: %f\n",
                   received_message.data.voting_result_msg.event_flag,
                   received_message.data.voting_result_msg.vote,
                   received_message.data.voting_result_msg.necessary_majority);
            task_ready_to_delete = true;
            break;
        default:
            break;
    }
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {}
