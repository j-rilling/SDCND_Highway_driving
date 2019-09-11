#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

class vehicle {
  public:
    vehicle(std::vector<double> stState);
    ~vehicle();
    std::vector<double> stateIn(double t);
  private:
    std::vector<double> startState;
    double s[3];
    double d[3];
};

#endif //VEHICLE_H