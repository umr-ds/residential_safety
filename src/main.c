#include <stdio.h>
#include <esp_now.h>
#include <esp_timer.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_mac.h"
#include "stdint.h"

struct dataMessage {
    uint32_t numRecvMessages;
    uint8_t loopCnt;
    bool finished;
};

const uint8_t mac_addresses[5][6] = {
        {0xE8, 0x9F, 0x6D, 0x33, 0x07, 0x7C},
        {0xE8, 0x9F, 0x6D, 0x32, 0x51, 0x68},
        {0xE8, 0x9F, 0x6D, 0x30, 0xDF, 0x94},
        {0xE8, 0x9F, 0x6D, 0x32, 0x49, 0x14},
        {0xE8, 0x9F, 0x6D, 0x30, 0xE2, 0xB4},
};


uint8_t own_mac[6];
static uint32_t numRecvMesgNode1 = 0;
static uint8_t loopCountNode1 = 0;
static uint32_t numRecvMesgNode2 = 0;
static uint8_t loopCountNode2 = 0;
static uint32_t numRecvMesgNode3 = 0;
static uint8_t loopCountNode3 = 0;
static uint32_t numRecvMesgNode4 = 0;
static uint8_t loopCountNode4 = 0;
static volatile bool acked = false;
static volatile bool finished_receivedNode1 = false;
static volatile bool finished_receivedNode2 = false;
static volatile bool finished_receivedNode3 = false;
static volatile bool finished_receivedNode4 = false;

int mac_addresses_equal(const uint8_t *mac1, const uint8_t *mac2) {
    // Compare each byte of the MAC addresses
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return 0; // MAC addresses are different
        }
    }
    return 1; // MAC addresses are the same
}


void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // If we are node 0
    if(mac_addresses_equal(own_mac, mac_addresses[0]))
    {
        if(mac_addresses_equal(recv_info->src_addr, mac_addresses[1])){
            if(len == sizeof(uint32_t)) numRecvMesgNode1++;
            else if(len == sizeof(bool) && *(bool*)data == true) {
                printf("%i, %li, 1\n", loopCountNode1, numRecvMesgNode1);
                loopCountNode1++;
                numRecvMesgNode1 = 0;
            }
        } else if (mac_addresses_equal(recv_info->src_addr, mac_addresses[2])) {
            if(len == sizeof(uint32_t)) numRecvMesgNode2++;
            else if(len == sizeof(bool) && *(bool*)data == true){
                printf("%i, %li, 2\n", loopCountNode2, numRecvMesgNode2);
                loopCountNode2++;
                numRecvMesgNode2 = 0;
            }
        } else if (mac_addresses_equal(recv_info->src_addr, mac_addresses[3])) {
            if(len == sizeof(uint32_t)) numRecvMesgNode3++;
            else if(len == sizeof(bool) && *(bool*)data == true){
                printf("%i, %li, 3\n", loopCountNode3, numRecvMesgNode3);
                loopCountNode3++;
                numRecvMesgNode3 = 0;
            }
        } else if(mac_addresses_equal(recv_info->src_addr, mac_addresses[4])) {
            if(len == sizeof(uint32_t)) numRecvMesgNode4++;
            else if(len == sizeof(bool) && *(bool*)data == true) {
                printf("%i, %li, 4\n", loopCountNode4, numRecvMesgNode4);
                loopCountNode4++;
                numRecvMesgNode4 = 0;
            }
        }
    }
}


void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {}

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

void initESPNOW(){
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    // Add peers
    for (int i = 0; i < 5; i++) {
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
        peerInfo.channel = 0;
        peerInfo.ifidx = ESP_IF_WIFI_STA;
        peerInfo.encrypt = false;
        memcpy(peerInfo.peer_addr, mac_addresses[i], sizeof(peerInfo.peer_addr));
        ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    }

}

void app_main(void)
{
    initWifi();
    initESPNOW();
    // If we are node 0
    if (mac_addresses_equal(own_mac, mac_addresses[0])) {
        while(1) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    } else {
        // If we are not node 0 send messages to node 0
        while(1) {
            // send out 1000 messages
            for(int i = 0; i<1000; i++){
                uint32_t dummy_data = 31123;
                esp_now_send(mac_addresses[0], (uint8_t*)&dummy_data, sizeof(uint32_t));
                vTaskDelay(50/portTICK_PERIOD_MS);
            }
            bool finished = true;
            esp_now_send(mac_addresses[0], (uint8_t*)&finished, sizeof(bool));
            vTaskDelay(4000/portTICK_PERIOD_MS);
        }
    }
}



