#include "voting.h"
#include "weights.h"

float votes[NUM_SENSORS];
uint8_t voted_sensors[NUM_SENSORS];

void init_votes(){
    for(int i = 0; i < NUM_SENSORS; i++){
        votes[i] = 0;
        voted_sensors[i] = 0;
    }
}


float calculate_water_leakage_vote() {
    initLeakageSensor();
    return (float)readLeakageSensor() * leakage_weight * node_weight_water_leakage;
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

Message start_voting(int event_flag) {
    float vote;
    Message msg = {};
    printf("called start voting method with event flag: %i\n", event_flag);
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            vote = (float)calculate_water_leakage_vote();
            msg.message_type = VOTING_MESSAGE_TYPE;
            msg.data.voting_msg.vote = vote;
            msg.data.voting_msg.event_flag = event_flag;
            return msg;
        default:
            return msg;
    }
}

void set_sensor_voted(int index){
    printf("Setting sensor voted on index %i\n", index);
    voted_sensors[index] = 1;
}

void set_vote(int index, float value){
    printf("Setting vote of index %i to value %f\n", index, value);
    votes[index] = value;
}

uint8_t check_votes(){
    for(int i = 0; i < NUM_SENSORS; i++){
        printf("Voted sensors on %i is %i\n", i, voted_sensors[i]);
        if(voted_sensors[i]==0) return 0;
    }
    return 1;
}

uint8_t calculate_decision(int event_flag){
    float sum_votes = 0;
    for(int i = 0; i < NUM_SENSORS; i++){
        printf("Vote on index: %i is %f\n", i, votes[i]);
        sum_votes+=votes[i];
    }
    printf("Sum votes: %f ", sum_votes);
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            if(sum_votes > NUM_SENSORS/2){
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


