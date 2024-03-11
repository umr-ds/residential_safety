#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H

#include "sensor_reader.h"

#define NUM_CALIBRATION_VALUES 300


#define MICROSEC_TO_SEC (1000*1000)
#define MESSAGE_WAITING_INTERVAL (5*MICROSEC_TO_SEC)
#define DEEPSLEEP_INTERVAL (60*MICROSEC_TO_SEC)
#define VOTING_TIMEOUT_INTERVAL (DEEPSLEEP_INTERVAL+MESSAGE_WAITING_INTERVAL+2*MICROSEC_TO_SEC)



RTC_DATA_ATTR float meanTemp = 0;


#endif //MASTERTHESIS_MAIN_H
