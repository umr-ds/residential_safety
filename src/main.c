#include <stdio.h>
#include "main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_timer.h"
#include "sensor_reader.h"
#include "communication.h"

Message message;
volatile int messageReceivedFlag = 0;
volatile int ISRFlag = GPIO_NUM_NC;

void ISR(void* arg) {
    gpio_num_t pin = (gpio_num_t)arg;
    switch (pin) {
        case PIR_SENSOR_PIN:
            ISRFlag = PIR_FLAG;
            break;
        case HALL_SENSOR_PIN:
            ISRFlag = HALL_FLAG;
            break;
        case LEAKAGE_SENSOR_PIN:
            ISRFlag = LEAKAGE_FLAG;
            break;
        default:
            break;

    }
};

void setup() {
    printf("Initializing Sensors\n");
    initSensors(ISR);
    calibrateCO(NUM_CALIBRATION_VALUES, &mean_CO, &stdDev_CO);
    calibrateOdor(NUM_CALIBRATION_VALUES, &mean_odor, &stdDev_odor);
    calibrateTemperature(NUM_CALIBRATION_VALUES, &mean_temperature, &stdDev_temperature);
    calibrateAccelerometer(NUM_CALIBRATION_VALUES, &mean_accel, &stdDev_accel);

    printf("CO Mean: %f\n", mean_CO);
    printf("CO StdDev: %f\n", stdDev_CO);
    printf("Odor Mean: %f\n", mean_odor);
    printf("Odor Std Dev: %f\n", stdDev_odor);
    printf("Temp Mean: %f\n", mean_temperature);
    printf("Temp Std Dev: %f\n", stdDev_temperature);
    printf("Accel Mean X: %f, Mean Y: %f, Mean Z: %f\n", mean_accel.x, mean_accel.y, mean_accel.z);
    printf("Accel StdDev X: %f, StdDev Y: %f, StdDev Z: %f\n", stdDev_accel.x, stdDev_accel.y, stdDev_accel.z);

}

void consensus_task(void *pvParameters) {

}

void measurement_task(void *pvParameters) {
    while(1) {
        printf("PIR: %i, Hall: %i, Leakage: %i\n", readPIRSensor(), readHallSensor(), readLeakageSensor());
        Acceleration acceleration = readAccelerometer();
        printf("CO Value: %lu\n", readCOSensor());
        printf("Odor Value: %lu\n", readOdorSensor());
        printf("Temperature Value: %f\n", readTemperatureSensor());
        printf("Accel X: %f, Accel Y: %f, Accel Z: %f\n", acceleration.x, acceleration.y, acceleration.z);
    }
}

void app_main(void)
{
    initWifi();
    initESPNOW(node_id, espnow_recv_cb, espnow_send_cb);
    setup();
    send_calibration_complete(node_id);
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
            .callback = NULL,
            .name = "timeout_timer"
    };

    uint64_t timeout_duration = 30 * 1000000;
    uint64_t start_time = esp_timer_get_time();
    esp_timer_create(&timer_args, &timer);
    uint64_t current_time = esp_timer_get_time();

    while (check_calibration_complete() == 0 || (current_time - start_time) <= timeout_duration) {
        current_time = esp_timer_get_time();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    xTaskCreatePinnedToCore(consensus_task, "consensus_task", 4096, NULL, 1, NULL,0);
    xTaskCreatePinnedToCore(measurement_task, "measurement_task", 4096, NULL, 1, NULL,1);
}

void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    message = *(Message *)data;
    messageReceivedFlag = 1;
}

void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

}