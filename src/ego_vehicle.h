#ifndef _EGO_VEHICLE_H_
#define _EGO_VEHICLE_H_

#include "PTG.h"
#include "helpers.h"
#include <vector>
#include <cmath>
#include <algorithm>

using std::vector;

class ego_vehicle {
  public:
    ego_vehicle();
    ~ego_vehicle();
    // This is a test method in order to test the comunication with the simulator
    vector<vector<double>> getCircularTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints);
    
    vector<vector<double>> keepLineTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsX, const vector<double> &mapsY);

    vector<vector<double>> trajXYToFrenet(const vector<double> &xPoints, const vector<double> &yPoints, 
        const vector<double> &thPoints, const vector<double> &mapsX, const vector<double> &mapsY);
    
    vector<vector<double>> trajFrenetToXY(const vector<double> &sPoints, const vector<double> &dPoints,
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY);
    
    vector<double> getThetasFromXY(const vector<double> &xPoints, const vector<double> &yPoints);
  private:
    PTG ptg;

    const unsigned int UPDATE_RATE = 10;
    unsigned int CURRENT_CYCLE; 

};

#endif // _EGO_VEHICLE_H_