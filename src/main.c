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


bool pending_reqests[MAX_NUM_SENSORS] = {false};
RTC_SLOW_ATTR volatile bool check_message_received = false;
RTC_SLOW_ATTR volatile bool voting_started = false;
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
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_0,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CO_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ODOR_SENSOR_ADC_CHANNEL, &config));

    ulp_wakeup_flag_co = 0;
    ulp_wakeup_flag_odor = 0;
    ulp_wakeup_flag_button = 0;
    ulp_high_thr_co = (int) mean_co + 3 * (int) stdDev_co;
    ulp_high_thr_odor = (int) mean_odor + 3 * (int) stdDev_odor;
    esp_deep_sleep_disable_rom_logging();
}

static void start_ulp_program(void) {
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

int get_num_nodes_available(){
    int cnt = 0;
    for(int i = 0; i < MAX_NUM_SENSORS; i++){
        if(nodes_available[i] == true) cnt++;
    }
    return cnt;
}

void update_nodes_available() {
    // Init array that holds available nodes with false
    for(int i = 0; i < MAX_NUM_SENSORS; i++) nodes_available[i] = false;
    int own_id = get_node_id(get_own_mac());
    // Set own entry of array to true
    nodes_available[own_id] = true;
    // Prepare check nodes message
    Message msg = {
            .src_node_id = own_id,
            .message_type = CHECK_NODES_MSG,
    };
    uint64_t start_time = esp_timer_get_time();
    // While loop should be at least as long as the deepsleep interval if in worst case a node
    // goes into deepsleep at the point we are checking if it is available
    while ((esp_timer_get_time() - start_time) <= VOTING_TIMEOUT_INTERVAL) {
        // if all nodes are available break to save power consumption
        if(get_num_nodes_available() == MAX_NUM_SENSORS) break;
        // loop through all node ids
        for (int i = 0; i < MAX_NUM_SENSORS; i++) {
            //printf("For loop counter is : %i\n", i);
            // if the id is ours or the node is already available, skip it
            if (i == own_id || nodes_available[i] == true) continue;
            else {
                // else send a message to it
                    send_message_to_node(msg, i);
                    vTaskDelay(100/portTICK_PERIOD_MS);
                }
            }
        }
        printf("\n\n");
}

bool check_pending_requests(bool* pending_requests, bool* available_nodes){
    for(int i = 0; i < MAX_NUM_SENSORS; i++){
        if(pending_requests[i] == false && available_nodes[i] == true ){
            printf("Pending request on %i is false and node is true. Waiting for it\n",i);
            return false;
        }
    }
    return true;
}

void voting_task(void *pvParameter){
    int event_flag = (int) pvParameter;
    printf("Voting task started for event flag: %u\n", event_flag);
    pending_reqests[get_node_id(get_own_mac())] = true;
    init_votes();
    Message msg = {
            .src_node_id = get_node_id(get_own_mac()),
            .message_type = VOTING_REQUEST_MESSAGE_TYPE,
            .data.voting_request_msg.event_flag = event_flag,
    };
    set_vote(get_node_id(get_own_mac()), calculate_vote(event_flag));
    set_sensor_voted(get_node_id(get_own_mac()));
    update_nodes_available();

    printf("Voting Task: Number of nodes available %i\n", get_num_nodes_available());
    while(1){
        if(check_votes(nodes_available, get_num_nodes_available()) == 1) {
            printf("Breaking because all nodes have voted\n");
            break;
        }
        // send message to each node that is available
        for(int i = 0; i < MAX_NUM_SENSORS; i++){
            // Skip if we are the current node
            if(get_node_id(get_own_mac()) == i ) continue;
            // Skip if the node is not available
            if(nodes_available[i] == false){
                printf("Skipping node %i, it is not available", i);
                continue;
            }
            // Skip if node has already voted
            if(get_sensor_voted(i) == true){
                printf("Skipping node %i, because it already voted\n", i);
                continue;
            }
            // Send message to node
            send_message_to_node(msg, i);
            // Give it some time to respond
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(check_pending_requests(pending_reqests, nodes_available) == false){
        printf("Waiting for pending requests\n");
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    int decision = calculate_decision(event_flag, get_num_nodes_available());
    /*msg.message_type = VOTING_RESULT_MESSAGE_TYPE;
    msg.data.voting_result_msg.event_flag = event_flag;
    msg.data.voting_result_msg.decision = decision;

    for(int i = 0; i < MAX_NUM_SENSORS; i++) {
        send_message_to_node(msg, i);
    }*/
    printf("Decision for flag: %i is %i\n", event_flag, decision);
    printf("Deleting voting task\n");
    voting_started = false;
    check_message_received = false;
    vTaskDelete(NULL);
}
void app_main() {

    initADCs();
    /*if(calibrated == false) {
        printf("Calibrating to preheat gas sensors. This may take 2 minutes, and is only done once the node was ejected from power\n");
        uint64_t start_time = esp_timer_get_time();
        while ((esp_timer_get_time() - start_time) <= CALIBRATION_INTERVAL){
            readOdorSensor();
            readCOSensor();
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
        calibrated = true;
    }*/
    calibrateOdor(NUM_CALIBRATION_VALUES, &mean_odor, &stdDev_odor);
    calibrateCO(NUM_CALIBRATION_VALUES, &mean_co, &stdDev_co);
    printf("Calibration finished: Mean CO: %f, std Dev Co: %f, Mean Odor: %f, std Dev Odor: %f\n", mean_co, stdDev_co, mean_odor, stdDev_odor);
    initWifi();
    initESPNOW(espnow_recv_cb, espnow_send_cb);
    // update nodes nur im voting task um downtime und strom zu sparen
    //initLED();

    //initI2CDriver();
    //lis3dh_init();
    //lis3dh_config_interrupt(10);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch(cause){
        case ESP_SLEEP_WAKEUP_TIMER:
            printf("Wakeup by timer\n");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            if((esp_sleep_get_ext1_wakeup_status() >> LEAKAGE_SENSOR_PIN) & 0x01) {
                printf("Wakeup from Leakage Pin\n");
                voting_started = true;
                xTaskCreate(&voting_task, "voting_task", 2048, (void*) WATER_LEAKAGE_FLAG, 5, &taskHandle);

            }
            if((esp_sleep_get_ext1_wakeup_status() >> ACCELEROMETER_INTR_PIN) & 0x01) {
                printf("Wakeup from Accelerometer Interrupt Pin\n");
                lis3dh_reset_interrupt();
                voting_started = true;
                xTaskCreate(&voting_task, "voting_task", 2048, (void*) SHOCK_FLAG, 5, &taskHandle);

            }
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            printf("ULP wakeup\n");
            ulp_wakeup_flag_odor &= UINT16_MAX;
            ulp_wakeup_flag_co &= UINT16_MAX;
            ulp_wakeup_flag_button &= UINT16_MAX;
            if(ulp_wakeup_flag_odor == 1){
                printf("Odor wakeup\n");
                voting_started = true;
                xTaskCreate(&voting_task, "voting_task", 2048, (void*) GAS_LEAKAGE_FLAG, 5, &taskHandle);
            }
            if(ulp_wakeup_flag_co == 1 ){
                printf("CO wakeup\n");
                voting_started = true;
                xTaskCreate(&voting_task, "voting_task", 2048, (void*) FIRE_FLAG, 5, &taskHandle);
            }
            if(ulp_wakeup_flag_button == 1) printf("Button wakeup\n");
        default:
            break;
    }
    if(voting_started == false) {
        uint64_t start_time = esp_timer_get_time();
        printf("Waiting for messages\n");
        while(esp_timer_get_time()-start_time <= MESSAGE_WAITING_INTERVAL){
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }
    while(voting_started == true || check_message_received == true) {
        vTaskDelay(100/portTICK_PERIOD_MS);
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

    ESP_ERROR_CHECK(rtc_gpio_init(BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(BUTTON_PIN));

    printf("Going to sleep mode\n");
    //init_ulp_program();

    //esp_sleep_enable_ulp_wakeup();
    //esp_sleep_enable_ext1_wakeup(((1ULL << LEAKAGE_SENSOR_PIN) | (1ULL << ACCELEROMETER_INTR_PIN)), ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_ext1_wakeup(((1ULL << LEAKAGE_SENSOR_PIN)), ESP_EXT1_WAKEUP_ANY_HIGH);
    //start_ulp_program();
    esp_deep_sleep(DEEPSLEEP_INTERVAL);

}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    //printf("Message received from node %i\n", get_node_id(recv_info->src_addr));
    Message received_message = *(Message *) data;
    switch (received_message.message_type) {
        case CHECK_NODES_MSG:
            printf("Received check nodes msg\n");
            check_message_received= true;
            Message ack_msg = {
                    .src_node_id = get_node_id(get_own_mac()),
                    .message_type = ACK_MESSAGE_TYPE,
            };
            //printf("Received Check Message from node: %i\n", get_node_id(recv_info->src_addr));
            ack_msg.data.ack_msg.message_flag = CHECK_NODES_MSG,
            send_message_to_node(ack_msg, get_node_id(recv_info->src_addr));
            break;
        case VOTING_REQUEST_MESSAGE_TYPE:
            if(taskHandle == NULL){
                printf("Task not present yet, creating it\n");
                voting_started = true;
                xTaskCreate(&voting_task, "voting_task", 2048, (void*) received_message.data.voting_request_msg.event_flag, 5, &taskHandle);
            }
            pending_reqests[get_node_id(recv_info->src_addr)] = true;
            float vote = get_vote(get_node_id(get_own_mac()));
            printf("Vote for event flag %i is: %f", received_message.data.voting_request_msg.event_flag, vote);
            Message voting_msg = {
                    .src_node_id = get_node_id(get_own_mac()),
                    .message_type = VOTING_ANSWER_MESSAGE_TYPE,
                    .data.voting_answer_msg.event_flag = received_message.data.voting_request_msg.event_flag,
                    .data.voting_answer_msg.vote = vote,
            };
            printf("Sending vote: %f\n", voting_msg.data.voting_answer_msg.vote);
            send_message_to_node(voting_msg, get_node_id(recv_info->src_addr));
            break;
        case VOTING_ANSWER_MESSAGE_TYPE:
            printf("Received Voting Answer with vote: %f\n", received_message.data.voting_answer_msg.vote);
            set_vote(get_node_id(recv_info->src_addr), received_message.data.voting_answer_msg.vote);
            set_sensor_voted(get_node_id(recv_info->src_addr));
            break;
        case VOTING_RESULT_MESSAGE_TYPE:
            if(received_message.data.voting_result_msg.decision == 1) printf("Nodes are accepting event :%i\n", received_message.data.voting_result_msg.event_flag);
            else printf("Nodes are not accepting event :%i\n", received_message.data.voting_result_msg.event_flag);
            voting_started = false;
            check_message_received = false;
            vTaskDelete(taskHandle);
            break;
        case ACK_MESSAGE_TYPE:
            if(received_message.data.ack_msg.message_flag == CHECK_NODES_MSG){
                printf("Received Check nodes Ack Message from node: %i\n", get_node_id(recv_info->src_addr));
                nodes_available[get_node_id(recv_info->src_addr)] = true;
            }
        default:
            break;
    }
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {}
