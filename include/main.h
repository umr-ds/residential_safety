#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H

#include "sensor_reader.h"

#define NUM_CALIBRATION_VALUES 300

RTC_DATA_ATTR float mean_odor = 0;
RTC_DATA_ATTR float stdDev_odor = 0;
RTC_DATA_ATTR float mean_co = 0;
RTC_DATA_ATTR float stdDev_co = 0;
RTC_DATA_ATTR Acceleration mean_accel;
RTC_DATA_ATTR Acceleration stdDev_accel;





#endif //MASTERTHESIS_MAIN_H
