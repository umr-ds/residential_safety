#include <stdio.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_mac.h"
#include "stdint.h"


const uint8_t mac_addresses[2][6] = {
        {0xE8, 0x9F, 0x6D, 0x33, 0x07, 0x7C},
        {0xE8, 0x9F, 0x6D, 0x32, 0x51, 0x68},
};

uint8_t own_mac[6];
static uint32_t numRecvMesg = 0;
static uint8_t loopCount = 0;
static volatile bool acked = false;

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
    if (mac_addresses_equal(recv_info->src_addr, mac_addresses[0]) == 1){
        acked = true;
    } else {
        if(len == sizeof(bool) && *(bool*)data == true){
            bool finished_received = 1;
            esp_now_send(mac_addresses[1], (uint8_t *)&finished_received, sizeof(bool));
            printf("%lu,%i\n", numRecvMesg, loopCount);
            numRecvMesg = 0;
            loopCount++;

        } else if(len == sizeof(uint32_t)){
            numRecvMesg++;
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
    for (int i = 0; i < 2; i++) {
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
    while(1){
        // Node 1 is transmitting , Node 0 receiving
        if(mac_addresses_equal(own_mac, mac_addresses[1])){
            for(int i = 1; i <= 1000; i++){
                uint32_t dummy_data = 0;
                esp_now_send(mac_addresses[0], (uint8_t *)&dummy_data, sizeof(uint32_t));
                vTaskDelay(10/portTICK_PERIOD_MS);
            }
            while(acked == false){
                bool finished = true;
                esp_now_send(mac_addresses[0], (uint8_t *)&finished, sizeof(bool));
                vTaskDelay(100/portTICK_PERIOD_MS);
            }
            acked = false;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}



