#include "vehicle.h"

vehicle::vehicle(std::vector<double> stState) {
    this->startState = stState;
}

vehicle::~vehicle() {
    
}

std::vector<double> vehicle::stateIn (double t) {
    std::vector<double> currentState;
    currentState.push_back(startState[0] + startState[1]*t + (startState[2]*t*t)/2);
    currentState.push_back(startState[1] + startState[2]*t);
    currentState.push_back(startState[2]);
    currentState.push_back(startState[3] + startState[4]*t + (startState[5]*t*t)/2);
    currentState.push_back(startState[4] + startState[5]*t);
    currentState.push_back(startState[5]);

    return currentState;
}