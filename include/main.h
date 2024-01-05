#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H

#include "sensor_reader.h"
#include "communication.h"

#define NUM_CALIBRATION_VALUES 300

uint8_t node_id = 0;

float mean_odor = 0;
float stdDev_odor = 0;
float mean_CO = 0;
float stdDev_CO = 0;
float mean_temperature = 0;
float stdDev_temperature = 0;
Acceleration mean_accel;
Acceleration stdDev_accel;

#endif //MASTERTHESIS_MAIN_H
