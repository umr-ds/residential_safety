#include "communication.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_mac.h"

//bool calibration_finished[5] = {false, false, false, false, false};
int calibration_finished[NUM_SENSORS];
uint8_t mac_addresses[5][6] = {
        {0xE8, 0x9F, 0x6D, 0x33, 0x07, 0x7C},
        {0xE8, 0x9F, 0x6D, 0x32, 0x51, 0x68},
        {0xE8, 0x9F, 0x6D, 0x30, 0xDF, 0x94},
        {0xE8, 0x9F, 0x6D, 0x32, 0x49, 0x14},
        {0xE8, 0x9F, 0x6D, 0x30, 0xE2, 0xB4},
};

uint8_t own_mac[6];

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
int mac_addresses_equal(const uint8_t* mac1, const uint8_t* mac2) {
    // Compare each byte of the MAC addresses
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return 0; // MAC addresses are different
        }
    }
    return 1; // MAC addresses are the same
}


int find_mac_address(const uint8_t mac_to_find[6]) {
    for (int i = 0; i < 5; i++) {
        if (memcmp(mac_addresses[i], mac_to_find, 6) == 0) {
            return i; // MAC address found at index i
        }
    }
    return -1; // MAC address not found
}

void initESPNOW(esp_now_recv_cb_t recvCallback, esp_now_send_cb_t sendCallback) {
    for(int i = 0; i < NUM_SENSORS; i++){
        calibration_finished[i] = 0;
    }
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recvCallback));
    ESP_ERROR_CHECK(esp_now_register_send_cb(sendCallback));
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if(mac_addresses_equal(own_mac, mac_addresses[i]) != 1) {
            esp_now_peer_info_t peerInfo;
            peerInfo.channel = 0;
            peerInfo.ifidx = ESP_IF_WIFI_STA;
            memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
            memcpy(peerInfo.peer_addr, mac_addresses[i], ESP_NOW_ETH_ALEN);
            ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
        }
    }
}

void send_calibration_complete() {
    int node_id = find_mac_address(own_mac);
    if(node_id == -1 || node_id >= NUM_SENSORS) {
        printf("Could not find mac address\n");
        return;
    }
    calibration_finished[node_id] = true;
    Message msg;
    memset(&msg, 0, sizeof(msg));
    memcpy(&msg.node_id, &node_id, sizeof(node_id));
    msg.message_type = CALIBRATION_MESSAGE_TYPE;
    msg.data.calibration_msg.calibration_finished = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (mac_addresses_equal(own_mac, mac_addresses[i]) != 1) {
            esp_now_send(mac_addresses[i], (uint8_t * ) & msg, sizeof(msg));
        }
    }

}

void set_calibration_finished(const uint8_t *mac_address){
    int node_id = find_mac_address(mac_address);
    if(node_id == -1 || node_id >= NUM_SENSORS) {
        printf("Could not find mac address\n");
        return;
    }
    calibration_finished[node_id] = true;
}

int check_calibration_complete() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        printf(calibration_finished[i] == 0 ? "Calibration at index %i is false\n" : "Calibration at index %i is true\n", i);
        if (calibration_finished[i] == 0) return 0;
    }
    return 1;
}

