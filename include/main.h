#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H

#include "sensor_reader.h"

#define NUM_CALIBRATION_VALUES 300


#define MICROSEC_TO_SEC (1000*1000)
#define MESSAGE_WAITING_INTERVAL (5*MICROSEC_TO_SEC)
#define DEEPSLEEP_INTERVAL (10*MICROSEC_TO_SEC)
#define VOTING_TIMEOUT_INTERVAL (DEEPSLEEP_INTERVAL+MESSAGE_WAITING_INTERVAL)

bool nodes_available[MAX_NUM_SENSORS] = {false};

RTC_DATA_ATTR float mean_odor = 0;
RTC_DATA_ATTR float stdDev_odor = 0;
RTC_DATA_ATTR float mean_co = 0;
RTC_DATA_ATTR float stdDev_co = 0;
RTC_DATA_ATTR Acceleration mean_accel;
RTC_DATA_ATTR Acceleration stdDev_accel;


#endif //MASTERTHESIS_MAIN_H
