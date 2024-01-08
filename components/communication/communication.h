#ifndef MASTERTHESIS_ESP_NOW_MANAGER_H
#define MASTERTHESIS_ESP_NOW_MANAGER_H

#define CALIBRATION_MESSAGE_TYPE 1
#define VOTING_MESSAGE_TYPE 2
#define DATA_MESSAGE_TYPE 3

#include <esp_now.h>
#include "stdint.h"

extern uint8_t mac_addresses[5][6];
extern uint8_t broadcast_address[6];


typedef struct Message {
    uint8_t message_type;
    uint8_t node_id;
    union {
        struct {
            bool calibration_finished;
        } calibration_msg;
        struct {
            bool voting_started;
            uint8_t vote_count;
        } voting_msg;
        struct {
            void *data;
            size_t data_size;
        } data_msg;
    } data;
} Message;


void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

void initWifi();

void initESPNOW(uint8_t node_id);

void send_calibration_complete(uint8_t node_id);

bool check_calibration_complete();


#endif //MASTERTHESIS_ESP_NOW_MANAGER_H