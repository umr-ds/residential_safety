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

float calculate_gas_or_fire_vote(event param){
    printf("Odor flag: %u, CO flag: %u\n", param.co_or_odor_event_flag.odor_flag, param.co_or_odor_event_flag.co_flag);
    float current_temp = readTemperatureSensor();
    printf("Current temp is: %f and mean Temp is: %f\n", current_temp, param.co_or_odor_event_flag.mean_temp);
    return 0.0;
    //return temp == true ? co_weight*co_flag + temperature_weight : odor_flag*odor_weight+co_flag*co_weight;
}


float calculate_intrusion_vote() {

    return pir_weight + hall_weight;
}

float calculate_shock_vote() {
    uint8_t regValue = lis3dh_read_register(0x31);
    if((regValue & 0x40) == 0x40) return accel_weight;
    else return 0.0;

}

float calculate_vote(event event) {
    float vote = 0.0;
    switch (event.event_flag) {
        case WATER_LEAKAGE_FLAG:
            return calculate_water_leakage_vote();
        case SHOCK_FLAG:
            return calculate_shock_vote();
        case FIRE_OR_GAS_FLAG:
            return calculate_gas_or_fire_vote(event);
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
    int cnt = 0;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if(voted_sensors[i] == true){
            cnt++;
        }
    }
    if(cnt == 1) {
        printf("Cnt is 1\n");
        for(int i = 0; i < MAX_NUM_SENSORS; i++){
            if(voted_sensors[i] == true && votes[i] >= 0.0){
                *required_majority = votes[i];
                *final_vote = votes[i];
                return 1;
            }
        }
    }
    for(int i = 0; i < MAX_NUM_SENSORS; i++) {
        if(node_weights[i] != 1.0) node_weights[i] = (node_weights[i]/MAX_NUM_SENSORS)*cnt;
        sum_votes += votes[i]*node_weights[i];
        printf("Vote of node %i  with weight %f is:  %f.\n", i, node_weights[i], votes[i]);
    }

    *final_vote = sum_votes;
    printf("Cnt is: %u\n", cnt);
    printf("Sum of votes is: %f\n", sum_votes);
    float majority;
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            majority = cnt/2.0;
            *required_majority = majority; //majority;
            if (sum_votes >=  majority && sum_votes > 0.0) return true;
            else return false;
        case SHOCK_FLAG:
            majority = cnt;
            *required_majority = majority;
            if (sum_votes >=  majority && sum_votes > 0.0) return true;
            else return false;
        case FIRE_OR_GAS_FLAG:
            majority = cnt/2.0;
            *required_majority = majority;
            if (sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;

        case INTRUSION_FLAG:
            majority = 1.0;
            *required_majority = majority;
            if(sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;
        default:
            return false;
    }
}


