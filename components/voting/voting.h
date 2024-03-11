#ifndef MASTERTHESIS_VOTING_H
#define MASTERTHESIS_VOTING_H

#include "sensor_reader.h"
#include "communication.h"

#define WATER_LEAKAGE_FLAG 1
#define INTRUSION_FLAG 2
#define SHOCK_FLAG 3
#define FIRE_OR_GAS_FLAG 4

#define NUM_SAMPLE_VALUES 500

typedef struct event event;

void init_votes();

void set_vote(int index, float value);

float get_vote(int index);

float get_own_node_weight(int event_flag);

float get_node_weight(int index);

void set_node_weight(int index, float value);

float calculate_vote(int event_flag);

void set_sensor_voted(int index);

bool get_sensor_voted(int index);

bool check_all_nodes_voted();

bool calculate_decision(int event_flag, float *final_vote, float *neccessary_majority);

#endif //MASTERTHESIS_VOTING_H

