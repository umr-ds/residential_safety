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

float calculate_gas_vote() {
    printf("Calculating Gas Vote\n");
    uint32_t mean = 0;
    for(int i = 0; i < 50; i++){
        mean+=readOdorSensor();
    }
    mean = mean/50;
    printf("Current odor mean is: %lu and calculated odor mean: %lu\n", mean, get_odor_mean());
    if(mean >= get_odor_mean()+200) return 1.0;
    else return 0.0;
}

float calculate_fire_vote() {
    printf("Calculating Fire Vote\n");
    uint32_t mean = 0;
    float vote = 0.0;
    for(int i = 0; i < 100; i++){
        mean += readCOSensor();
    }
    mean = mean/100;
    if(mean+200 >= get_co_mean() ) vote += co_weight;
    if(readTemperatureSensor() >= get_temperature_mean() + 3.0) vote+=temperature_weight;
    return vote;
}

float calculate_intrusion_vote() {
    initHallSensor();
    initPIRSensor();
    int movement = 0;
    for(int i = 0; i < 500; i++){
        if(readPIRSensor()==1){
            movement = 1;
            break;
        }
    }
    float vote = pir_weight*movement + hall_weight * readHallSensor();
    return vote;
}

float calculate_shock_vote() {
    uint8_t regValue = lis3dh_read_register(0x31);
    if ((regValue & 0x40) == 0x40) return accel_weight;
    else return 0.0;

}

float calculate_vote(int event_flag) {
    float vote = 0.0;
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            return calculate_water_leakage_vote();
        case SHOCK_FLAG:
            return calculate_shock_vote();
        case INTRUSION_FLAG:
            return calculate_intrusion_vote();
        case FIRE_OR_GAS_FLAG:
            if(readTemperatureSensor() >= get_temperature_mean() + 3.0) return calculate_fire_vote();
            else return calculate_gas_vote();
        default:
            return vote;
    }
}

void set_sensor_voted(int index) {
    voted_sensors[index] = true;
}

bool get_sensor_voted(int index) {
    return voted_sensors[index];
}

void set_vote(int index, float value) {
    votes[index] = value;
}

float get_vote(int index) {
    return votes[index];
}

float get_own_node_weight(int event_flag) {
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            return node_weight_water_leakage;
        case SHOCK_FLAG:
            return node_weight_shock;
        case INTRUSION_FLAG:
            return node_weight_shock;
        case FIRE_OR_GAS_FLAG:
            return node_weight_gas_or_fire;
        default:
            return 0.0;
    }
}

void set_node_weight(int index, float value) {
    node_weights[index] = value;
}

float get_node_weight(int index) {
    return node_weights[index];
}

bool check_all_nodes_voted() {
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if (voted_sensors[i] == false) return false;
    }
    return true;
}

bool calculate_decision(int event_flag, float *final_vote, float *required_majority) {
    float sum_votes = 0;
    int cnt = 0;
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if (voted_sensors[i] == true) {
            printf("Sensor %i voted with vote: %f\n", i, votes[i]);
            cnt++;
        }
    }
    if (cnt == 1) {
        int voted_index = -1;
        for (int i = 0; i < MAX_NUM_SENSORS; i++) {
            if (voted_sensors[i] == true && votes[i] > 0.0) voted_index = i;
        }
        if(voted_index == -1) {
            *required_majority = 1.0;
            *final_vote = 0;
            return 0;
        } else {
            if(event_flag == INTRUSION_FLAG) {
                *required_majority = 1.0;
                *final_vote = votes[voted_index];
                return 0;
            } else {
                *required_majority = votes[voted_index];
                *final_vote = votes[voted_index];
                return 1;
            }
        }
    }
    int adjusted_sum_votes = 0;
    int node_weights_copy[MAX_NUM_SENSORS] = {0};

    for(int i = 0; i < MAX_NUM_SENSORS; i++){
        adjusted_sum_votes+=node_weights[i];
    }

    for(int i = 0; i < MAX_NUM_SENSORS; i++){
        node_weights_copy[i] = node_weights[i]* (5/adjusted_sum_votes);
        sum_votes += votes[i]*node_weights_copy[i];
    }

    //for (int i = 0; i < MAX_NUM_SENSORS; i++) {
    //    //if (node_weights[i] != 1.0) node_weights[i] = (node_weights[i] / MAX_NUM_SENSORS) * cnt;
    //    sum_votes += votes[i] * node_weights[i];
    //}

    *final_vote = sum_votes;
    float majority;
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            //majority = adjusted_sum_votes / 2.0;
            majority = MAX_NUM_SENSORS / 2.0;
            *required_majority = majority; //majority;
            if (sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;
        case SHOCK_FLAG:
            //majority = cnt;
            majority = MAX_NUM_SENSORS;
            *required_majority = majority;
            if (sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;
        case INTRUSION_FLAG:
            majority = 1.0;
            *required_majority = majority;
            if (sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;
        case FIRE_OR_GAS_FLAG:
            //majority = adjusted_sum_votes/2.0;
            majority = MAX_NUM_SENSORS / 2.0;
            *required_majority = majority;
            if(sum_votes >= majority && sum_votes > 0) return true;
            else return false;
        default:
            return false;
    }
}


