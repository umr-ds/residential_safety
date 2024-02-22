//
// Created by Kevin on 29.01.2024.
//

#include "sensor_reader.h"

#ifndef MASTERTHESIS_WEIGHTS_H
#define MASTERTHESIS_WEIGHTS_H

RTC_DATA_ATTR float pir_weight = 1.0;
RTC_DATA_ATTR float hall_weight = 1.0;
RTC_DATA_ATTR float co_weight = 1.0;
RTC_DATA_ATTR float odor_weight = 1.0;
RTC_DATA_ATTR float accel_weight = 1.0;
RTC_DATA_ATTR float temperature_weight = 1.0;

RTC_DATA_ATTR double node_weight_water_leakage = 1.0;
RTC_DATA_ATTR float node_weight_fire = 1.0;
RTC_DATA_ATTR float node_weight_intrusion = 1.0;
RTC_DATA_ATTR float node_weight_gas_leakage = 1.0;
RTC_DATA_ATTR float node_weight_shock = 1.0;

#endif //MASTERTHESIS_WEIGHTS_H
