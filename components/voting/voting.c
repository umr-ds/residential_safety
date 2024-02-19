#include "voting.h"
#include "weights.h"

float votes[MAX_NUM_SENSORS];
bool voted_sensors[MAX_NUM_SENSORS];

void init_votes() {
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        votes[i] = 0;
        voted_sensors[i] = 0;
    }
}

float calculate_water_leakage_vote() {
    initLeakageSensor();
    return (float) readLeakageSensor() * leakage_weight * node_weight_water_leakage;
}

int calculate_gas_leakage_vote() {

    return odor_weight + co_weight;
}

int calculate_intrusion_vote() {

    return pir_weight + hall_weight;
}

int calculate_fire_vote() {

    return co_weight + temperature_weight;
}

int calculate_shock_vote() {

    return accel_weight;
}

float calculate_vote(int event_flag) {
    float vote = 0.0;
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            return calculate_water_leakage_vote();
        case SHOCK_FLAG:
            return calculate_shock_vote();
        case FIRE_FLAG:
            return calculate_fire_vote();
        case GAS_LEAKAGE_FLAG:
            return calculate_gas_leakage_vote();
        default:
            return vote;
    }
}

void set_sensor_voted(int index) {
    printf("Setting sensor voted at index %i to 1\n", index);
    voted_sensors[index] = true;
}

bool get_sensor_voted(int index){
    return voted_sensors[index];
}

void set_vote(int index, float value) {
    printf("Setting vote at index %i to value: %f\n", index, value);
    votes[index] = value;
}

bool check_votes(const bool* nodes_available, int num_nodes_available) {
    int cnt = 0;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        // Skip sensors that are not available
        if(nodes_available[i] == false){
            printf("Skipping node %i since it is not available\n", i);
            continue;
        }
        if (voted_sensors[i] == true){
            printf("Node %i has voted, incrementing counter\n", i);
            cnt++;
        }
    }
    if(cnt == num_nodes_available) return true;
    else return false;
}

uint8_t calculate_decision(int event_flag, int num_nodes_available) {
    float sum_votes = 0;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        sum_votes += votes[i];
    }
    printf("Sum of votes is: %f\n", sum_votes);
    printf("Necessary majority: %f\n", num_nodes_available/2.0);
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            if (sum_votes >=  num_nodes_available / 2.0) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case SHOCK_FLAG:
            if (sum_votes >= num_nodes_available / 2) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case FIRE_FLAG:
            if (sum_votes >= num_nodes_available / 2) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case GAS_LEAKAGE_FLAG:
            if (sum_votes >= num_nodes_available / 2) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        default:
            return 0;
    }
}


