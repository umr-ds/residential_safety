#ifndef MASTERTHESIS_VOTING_H
#define MASTERTHESIS_VOTING_H

#include "sensor_reader.h"
#include "communication.h"

#define WATER_LEAKAGE_FLAG 1
#define INTRUSION_FLAG 2
#define FIRE_FLAG 3
#define SHOCK_FLAG 4
#define GAS_LEAKAGE_FLAG 5

#define NUM_SAMPLE_VALUES 500


void init_votes();

void set_vote(int index, float value);

float get_vote(int index);

float calculate_vote(int event_flag);

void set_sensor_voted(int index);

bool get_sensor_voted(int index);

bool check_votes(const bool* nodes_available, int num_nodes_available);

uint8_t calculate_decision(int event_flag, int num_nodes_available);

#endif //MASTERTHESIS_VOTING_H

