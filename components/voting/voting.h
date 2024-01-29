#ifndef MASTERTHESIS_VOTING_H
#define MASTERTHESIS_VOTING_H

#include "sensor_reader.h"
#include "communication.h"

#define WATER_LEAKAGE_FLAG 1
#define INTRUSION_FLAG 2
#define FIRE_FLAG 3
#define SHOCK_FLAG 4
#define GAS_LEAKAGE_FLAG 5

float calculate_water_leakage_vote();

int calculate_gas_leakage_vote();

int calculate_intrusion_vote();

int calculate_fire_vote();

int calculate_shock_vote();

void init_votes();

void set_vote(int index, float value);

Message start_voting(int event_flag);

void set_sensor_voted(int index);

uint8_t check_votes();

uint8_t calculate_decision(int event_flag);

#endif //MASTERTHESIS_VOTING_H

