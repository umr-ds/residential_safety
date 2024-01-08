#include "communication.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "main.h"

bool calibration_finished[5] = {false, false, false, false, false};
uint8_t mac_addresses[5][6] = {
        {0xE8, 0x9F, 0x6D, 0x33, 0x07, 0x7C},
        {0xE8, 0x9F, 0x6D, 0x32, 0x51, 0x68},
        {0xE8, 0x9F, 0x6D, 0x30, 0xDF, 0x94},
        {0xE8, 0x9F, 0x6D, 0x32, 0x49, 0x14},
        {0xE8, 0x9F, 0x6D, 0x30, 0xE2, 0xB4},
};

uint8_t broadcastAddress[6] = {0xFF,
                               0xFF,
                               0xFF,
                               0xFF,
                               0xFF,
                               0xFF};


void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    receivedMessage = (Message *)data;
    messageReceivedFlag = 1;
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

void initWifi() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

}

void initESPNOW(uint8_t node_id) {
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    for (int i = 0; i < 5; i++) {
        if (i != node_id) {
            esp_now_peer_info_t peerInfo;
            peerInfo.channel = 0;
            peerInfo.ifidx = ESP_IF_WIFI_STA;
            memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
            memcpy(peerInfo.peer_addr, mac_addresses[i], ESP_NOW_ETH_ALEN);
            ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
        }
    }
    esp_now_peer_info_t peerInfo;
    peerInfo.channel = 0;
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
    memcpy(peerInfo.peer_addr, broadcastAddress, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

void send_calibration_complete(uint8_t node_id) {
    calibration_finished[node_id] = true;
    Message msg;
    memset(&msg, 0, sizeof(msg));
    memcpy(&msg.node_id, &node_id, sizeof(node_id));
    msg.message_type = CALIBRATION_MESSAGE_TYPE;
    msg.data.calibration_msg.calibration_finished = true;
    for (int i = 0; i < 5; i++) {
        if (i != node_id) {
            esp_now_send(mac_addresses[i], (uint8_t * ) & msg, sizeof(msg));
        }
    }

}

bool check_calibration_complete() {
    for (int i = 0; i < 5; i++) {
        if (calibration_finished[i] == false) return false;
    }
    return true;
}



