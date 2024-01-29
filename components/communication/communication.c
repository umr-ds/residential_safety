#include "communication.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_mac.h"

const uint8_t mac_addresses[5][6] = {
        {0xE8, 0x9F, 0x6D, 0x33, 0x07, 0x7C},
        {0xE8, 0x9F, 0x6D, 0x32, 0x51, 0x68},
        {0xE8, 0x9F, 0x6D, 0x30, 0xDF, 0x94},
        {0xE8, 0x9F, 0x6D, 0x32, 0x49, 0x14},
        {0xE8, 0x9F, 0x6D, 0x30, 0xE2, 0xB4},
};

uint8_t own_mac[6];


int mac_addresses_equal(const uint8_t *mac1, const uint8_t *mac2) {
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

uint8_t *get_own_mac() {
    return own_mac;
}

const uint8_t *get_mac_address(uint8_t node_id) {
    return mac_addresses[node_id];
}

uint8_t get_node_id(uint8_t *mac_address) {
    return find_mac_address(mac_address);
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
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

}

void initESPNOW(esp_now_recv_cb_t recvCallback, esp_now_send_cb_t sendCallback) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recvCallback));
    ESP_ERROR_CHECK(esp_now_register_send_cb(sendCallback));
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (mac_addresses_equal(own_mac, mac_addresses[i]) != 1) {
            esp_now_peer_info_t peerInfo;
            peerInfo.channel = 0;
            peerInfo.ifidx = ESP_IF_WIFI_STA;
            memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
            memcpy(peerInfo.peer_addr, mac_addresses[i], ESP_NOW_ETH_ALEN);
            ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
        }
    }

}

void send_ack_message(uint8_t event_flag, uint8_t dest_node_id) {
    Message message = {
            .src_node_id = find_mac_address(own_mac),
            .message_type = ACK_MESSAGE_TYPE,
            .data.ack_msg.event_flag = event_flag
    };
    esp_now_send(mac_addresses[dest_node_id], (uint8_t *) &message, sizeof(message));
}

void send_message_to_node(Message message, uint8_t dest_node_id) {
    esp_now_send(mac_addresses[dest_node_id], (uint8_t *) &message, sizeof(message));
}

void broadcast_message(Message message) {
    message.src_node_id = find_mac_address(own_mac);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (mac_addresses_equal(own_mac, mac_addresses[i]) != 1) {
            esp_now_send(mac_addresses[i], (uint8_t *) &message, sizeof(message));
        }
    }
}



