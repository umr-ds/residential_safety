#include "voting.h"
#include "weights.h"
#include "communication.h"

// Contains the vote of each sensor
float votes[MAX_NUM_SENSORS];
// Indicates if a specific sensor has voted
bool voted_nodes[MAX_NUM_SENSORS];
// Contains the node weight of each sensor
float node_weights[MAX_NUM_SENSORS];

void init_votes() {
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        votes[i] = 0;
        voted_nodes[i] = false;
    }
}

// Returns the vote for the water leakage scenario
// Initialize sensor and return if it detects water (1.0) or not (0.0)
float calculate_water_leakage_vote() {
    init_leakage_sensor();
    return (float) read_leakage_sensor();
}

// Returns the vote for the gas leakage scenario
// Read 50 values, calculate mean out of it and compare it to the threshold
// Returns 1.0 if mean is above threshold, 0.0 otherwise
float calculate_gas_vote() {
    uint32_t mean = 0;
    for (int i = 0; i < 50; i++) {
        mean += read_odor_sensor();
    }
    mean = mean / 50;
    if (mean >= get_odor_mean() + 200) return 1.0;
    else return 0.0;
}

// Returns the vote for the fire scenario
// Read 50 values, calculate mean out of it and compare it to the threshold
// Additionally compares if the temperature rise is 3.0 degrees higher than the mean temperature to indicate a potential fire
// Returns 0.5 if mean value is above the threshold, 1.0 if the temperature is also above threshold and 0.0 if nothing is above the threshold
float calculate_fire_vote() {
    uint32_t mean = 0;
    float vote = 0.0;
    for (int i = 0; i < 50; i++) {
        mean += read_co_sensor();
    }
    mean = mean / 50;
    if (mean + 200 >= get_co_mean()) vote += co_weight;
    if (read_temperature_sensor() >= get_temperature_mean() + 3.0) vote += temperature_weight;
    return vote;
}

// Returns the vote for the intrusion scenario
// Read 500 values from the PIR sensor to ensure the delayed detection of the sensor
// Adds 0.5 to the vote if the PIR sensor detected a movement
// Adds 0.5 to the vote if the hall sensor detected a magnet => open door
float calculate_intrusion_vote() {
    init_hall_sensor();
    init_pir_sensor();
    int movement = 0;
    for (int i = 0; i < 500; i++) {
        if (read_pir_sensor() == 1) {
            movement = 1;
            break;
        }
    }
    float vote = pir_weight * movement + hall_weight * read_hall_sensor();
    return vote;
}

// Returns the vote for the earthquake scenario
// Read out the interrupt status register of the accelerometer
// Return the accelerometer weight if one axis experienced a value above the defined threshold, 0.0 otherwise
float calculate_shock_vote() {
    uint8_t regValue = lis3dh_read_register(0x31);
    if ((regValue & 0x40) == 0x40) return accel_weight;
    else return 0.0;

}

// Returns the vote of the appropriate 'event_flag'
// Calls the method for vote calculation associated to the 'event_flag'
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
            if (read_temperature_sensor() >= get_temperature_mean() + 3.0) return calculate_fire_vote();
            else return calculate_gas_vote();
        default:
            return vote;
    }
}

// Set the value of the voted_nodes array at index to true
void set_node_voted(int index) {
    voted_nodes[index] = true;
}

// Returns if the node at index has voted
bool get_node_voted(int index) {
    return voted_nodes[index];
}

// Set vote at index to value
void set_vote(int index, float value) {
    votes[index] = value;
}

// Returns the vote of node at index 'index'
float get_vote(int index) {
    return votes[index];
}

// Returns the node weight associated with the 'event_flag'
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

// Set the node weight of the node 'index' to 'value'
// Only used for voting, does not change the local node weight of a node
void set_node_weight(int index, float value) {
    node_weights[index] = value;
}

// Returns the node weight of node 'index'
float get_node_weight(int index) {
    return node_weights[index];
}

// Check if all nodes (MAX_NUM_SENSORS) have voted
// Returns true if MAX_NUM_SENSORS have voted, false otherwise
bool check_all_nodes_voted() {
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if (voted_nodes[i] == false) return false;
    }
    return true;
}

// Calculate the decision of the voting phase for the scenario indicated by 'event_flag'
// Returns true if the sum of all votes is higher than the majority, otherwise false
// Returns the sum of all votes as 'final_vote' and the required majority for the scenario as 'required_majority'
bool calculate_decision(int event_flag, float *final_vote, float *required_majority) {
    float sum_votes = 0;
    int cnt = 0;
    // Count the number of nodes that have voted
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        if (voted_nodes[i] == true) {
            cnt++;
        }
    }
    // Adjust the output if only one node has voted
    if (cnt == 1) {
        int voted_index = -1;
        for (int i = 0; i < MAX_NUM_SENSORS; i++) {
            if (voted_nodes[i] == true && votes[i] > 0.0) voted_index = i;
        }
        if (voted_index == -1) {
            *required_majority = 1.0;
            *final_vote = 0;
            return 0;
        } else {
            if (event_flag == INTRUSION_FLAG) {
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

    // Sum up the node weights of each node
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        adjusted_sum_votes += node_weights[i];
    }

    // Rebalance the node weights in case less than 'MAX_NUM_SENSORS' have voted
    // Multiply the vote with the adjusted node weight and add it to 'sum_votes'
    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
        node_weights_copy[i] = node_weights[i] * (MAX_NUM_SENSORS / adjusted_sum_votes);
        sum_votes += votes[i] * node_weights_copy[i];
    }

    *final_vote = sum_votes;
    float majority;
    switch (event_flag) {
        case WATER_LEAKAGE_FLAG:
            majority = MAX_NUM_SENSORS / 2.0;
            *required_majority = majority;
            if (sum_votes >= majority && sum_votes > 0.0) return true;
            else return false;
        case SHOCK_FLAG:
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
            majority = MAX_NUM_SENSORS / 2.0;
            *required_majority = majority;
            if (sum_votes >= majority && sum_votes > 0) return true;
            else return false;
        default:
            return false;
    }
}