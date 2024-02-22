#include "voting.h"
#include "weights.h"
#include "communication.h"

float votes[MAX_NUM_SENSORS];
bool voted_sensors[MAX_NUM_SENSORS];
float node_weights[MAX_NUM_SENSORS];

void init_votes() {
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        votes[i] = 0;
        voted_sensors[i] = 0;
    }
}

float calculate_water_leakage_vote() {
    initLeakageSensor();
    return (float) readLeakageSensor();
}

float calculate_intrusion_vote() {

    return pir_weight + hall_weight;
}

float calculate_shock_vote() {
    uint8_t regValue = lis3dh_read_register(0x31);
    lis3dh_reset_interrupt();
    if((regValue & 0x40) == 0x40) {
        return accel_weight;
    } else {
        return 0.0;
    }
}

float calculate_vote(event event) {
    float vote = 0.0;
    switch (event.event_flag) {
        case WATER_LEAKAGE_FLAG:
            return calculate_water_leakage_vote();
        case SHOCK_FLAG:
            return calculate_shock_vote();
        case FIRE_OR_GAS_FLAG:
            return 0;
            // look for the other gas sensor and temperature. If temperature is rising, its fire, otherwise its gas
            //if(event.co_or_odor_event_flag.co_flag == true  event.co_or_odor_event_flag.co_flag)
            return vote;
        case INTRUSION_FLAG:
            return calculate_intrusion_vote();
        default:
            return vote;
    }
}

void set_sensor_voted(int index) {
    voted_sensors[index] = true;
}

bool get_sensor_voted(int index){
    return voted_sensors[index];
}

void set_vote(int index, float value) {
    votes[index] = value;
}

float get_vote(int index){
    return votes[index];
}

float get_own_node_weight(int event_flag){
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            return node_weight_water_leakage;
        case FIRE_OR_GAS_FLAG:
            return node_weight_gas_leakage;
        case SHOCK_FLAG:
            return node_weight_shock;
        case INTRUSION_FLAG:
            return node_weight_shock;
        default:
            return 0.0;
    }
}

void set_node_weight(int index, float value){
    node_weights[index] = value;
}

float get_node_weight(int index){
    return node_weights[index];
}

bool check_all_nodes_voted(){
    for(int i = 0; i < MAX_NUM_SENSORS; i++){
        if(voted_sensors[i] == false) return false;
    }
    return true;
}

bool calculate_decision(int event_flag, float* final_vote, float* required_majority) {
    float sum_votes = 0;
    float majority;
    float normalized_majority = 0;
    int cnt = 0;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if(voted_sensors[i] == true){
            cnt++;
        }
    }
    for(int i = 0; i < MAX_NUM_SENSORS; i++) {
        float normalized_vote = (node_weights[i]/MAX_NUM_SENSORS)*cnt;
        sum_votes += normalized_vote * votes[i];
        printf("Normalized vote of node with weight %f is: %i is %f. Original vote was: %f\n", node_weights[i], i, normalized_vote, votes[i]);
    }
    normalized_majority = (float)(1.0/MAX_NUM_SENSORS) * cnt;
    printf("Noramlized majority is %f\n", normalized_majority);
    *final_vote = sum_votes;
    *required_majority = normalized_majority; //majority;
    printf("Sum of votes is: %f\n", sum_votes);
    printf("Necessary majority: %f\n", normalized_majority);
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            if (sum_votes >=  normalized_majority && sum_votes > 0.0) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case SHOCK_FLAG:
            if (sum_votes >=  *required_majority && sum_votes > 0.0) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case FIRE_OR_GAS_FLAG:
            if (sum_votes >= *required_majority && sum_votes > 0.0) {
                printf("Accepting\n");
                return 1;
            } else {
                printf("No majority reached\n");
                return 0;
            }
        case INTRUSION_FLAG:
            if(sum_votes >= *required_majority && sum_votes > 0.0){
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


