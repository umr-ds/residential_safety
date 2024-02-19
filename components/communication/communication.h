#ifndef MASTERTHESIS_ESP_NOW_MANAGER_H
#define MASTERTHESIS_ESP_NOW_MANAGER_H

#include <esp_now.h>
#include "stdint.h"

#define MAX_NUM_SENSORS 3

#define CHECK_NODES_MSG 1
#define VOTING_REQUEST_MESSAGE_TYPE 2
#define VOTING_ANSWER_MESSAGE_TYPE 3
#define VOTING_RESULT_MESSAGE_TYPE 4
#define ACK_MESSAGE_TYPE 5




extern const uint8_t mac_addresses[5][6];

typedef struct Message {
    int message_type;
    uint8_t src_node_id;
    union {
        struct {
            int event_flag;
        } voting_request_msg;
        struct {
            int event_flag;
            float vote;
        } voting_answer_msg;
        struct {
            int event_flag;
            uint8_t decision;
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