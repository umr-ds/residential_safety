#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H

#include "sensor_reader.h"

#define NUM_CALIBRATION_VALUES 300
#define NUM_SENSORS 2

#define LEAKAGE_FLAG 1
#define PIR_FLAG 2
#define HALL_FLAG 3


RTC_DATA_ATTR uint8_t node_id = 0;
RTC_DATA_ATTR float mean_odor = 0;
RTC_DATA_ATTR float stdDev_odor = 0;
RTC_DATA_ATTR float mean_co = 0;
RTC_DATA_ATTR float stdDev_co = 0;
RTC_DATA_ATTR float mean_temperature = 0;
RTC_DATA_ATTR float stdDev_temperature = 0;
RTC_DATA_ATTR Acceleration mean_accel;
RTC_DATA_ATTR Acceleration stdDev_accel;

#endif //MASTERTHESIS_MAIN_H
