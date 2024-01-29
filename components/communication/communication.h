#ifndef MASTERTHESIS_ESP_NOW_MANAGER_H
#define MASTERTHESIS_ESP_NOW_MANAGER_H

#include <esp_now.h>
#include "stdint.h"

#define SYNCED_MESSAGE_TYPE 1
#define VOTING_MESSAGE_TYPE 2
#define DATA_MESSAGE_TYPE 3
#define ACK_MESSAGE_TYPE 4

#define NUM_SENSORS 2

extern const uint8_t mac_addresses[5][6];


typedef struct Message {
    uint8_t message_type;
    uint8_t src_node_id;
    union {
        struct {
            uint8_t event_flag;
            float vote;
        } voting_msg;
        struct {
            uint8_t all_nodes_synced;
        } sync_msg;
        struct {
            uint8_t event_flag;
        } ack_msg;
        struct {
            void *data;
            size_t data_size;
        } data_msg;
    } data;
} Message;

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);


void initWifi();

void initESPNOW(esp_now_recv_cb_t recvCallback, esp_now_send_cb_t sendCallback);

uint8_t* get_own_mac();
const uint8_t* get_mac_address(uint8_t node_id);
uint8_t get_node_id(uint8_t *mac_address);


void broadcast_message(Message msg);
void send_ack_message(uint8_t event_flag, uint8_t dest_node_id);
void send_message_to_node(Message message, uint8_t dest_node_id);


#endif //MASTERTHESIS_ESP_NOW_MANAGER_H