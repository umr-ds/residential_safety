#ifndef MASTERTHESIS_ESP_NOW_MANAGER_H
#define MASTERTHESIS_ESP_NOW_MANAGER_H

#include <esp_now.h>
#include "stdint.h"

#define MAX_NUM_SENSORS 5

#define VOTING_REQUEST_MESSAGE_TYPE 1
#define VOTING_ANSWER_MESSAGE_TYPE 2
#define VOTING_RESULT_MESSAGE_TYPE 3
#define ALARM_MODE_MESSAGE_TYPE 4



extern const uint8_t mac_addresses[5][6];

typedef struct event {
    uint8_t event_flag;
    struct {
        bool odor_flag;
        bool co_flag;
    } co_or_odor_event_flag;
} event;

typedef struct Message {
    int message_type;
    union {
        struct {
            event event;
        } voting_request_msg;
        struct {
            int event_flag;
            float vote;
            float vote_weight;
        } voting_answer_msg;
        struct {
            int event_flag;
            bool decision;
            float vote;
            float necessary_majority;
        } voting_result_msg;
        struct {
            int message_flag;
        } ack_msg;
    } data;
} Message;

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

void initWifi();

void initESPNOW(esp_now_recv_cb_t recvCallback, esp_now_send_cb_t sendCallback);

uint8_t *get_own_mac();

const uint8_t *get_mac_address(uint8_t node_id);

uint8_t get_node_id(uint8_t *mac_address);

void send_message_to_node(Message message, uint8_t dest_node_id);


#endif //MASTERTHESIS_ESP_NOW_MANAGER_H