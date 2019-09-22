#ifndef _EGO_VEHICLE_H_
#define _EGO_VEHICLE_H_

#include "PTG.h"
#include "helpers.h"
#include <vector>
#include <map>
#include <list>
#include <cmath>
#include <algorithm>
#include "spline.h"

using std::vector;
using std::list;

struct trajectoryInfo {
        unsigned int start_lane;
        unsigned int intended_lane;
        unsigned int final_lane;
        double start_s;
        double end_s;
        double velocity;
        double acceleration;
        string state;
};  

class ego_vehicle {
  public:
    ego_vehicle();
    ~ego_vehicle();
    // This is a test method in order to test the comunication with the simulator
    vector<vector<double>> getCircularTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints);
    
    vector<vector<double>> PTGkeepLineTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY);

    vector<vector<double>> SplineTraj(double x0, double y0, double th0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY);

    void updateTrajectory(const vector<double> &previousXpoints, double s0, double endPathS, const vector<vector<double>> &otherCars);

    vector<vector<double>> trajXYToFrenet(const vector<double> &xPoints, const vector<double> &yPoints, 
        const vector<double> &thPoints, const vector<double> &mapsX, const vector<double> &mapsY);
    
    vector<vector<double>> trajFrenetToXY(const vector<double> &sPoints, const vector<double> &dPoints,
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY);

    vector<double> getVelocity(const vector<double> &posPoints);

    vector<double> getAcceleration(const vector<double> &velPoints);
    
    vector<double> getThetasFromXY(const vector<double> &xPoints, const vector<double> &yPoints);

    bool getVehicleAhead(double lastPathSize, const vector<vector<double>> &otherVehicles, 
                        unsigned int lane, vector<double> &vehicleFound);

    bool getVehicleBehind(const vector<vector<double>> &otherVehicles, 
                          unsigned int lane, vector<double> &vehicleFound);

    vector<double> getKinematicsOfLane(double lastPathSize, const vector<vector<double>> &otherVehicles, unsigned int lane);

    trajectoryInfo keepLaneTraj(double lastPathSize, const vector<vector<double>> &otherVehicles);
    trajectoryInfo prepLaneChangeTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles);
    trajectoryInfo laneChangeTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles);
    trajectoryInfo generateStateTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles);
    vector<string> possibleNextStates();
    
    // Cost functions
    double avgLaneSpeedCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles); //
    double NextCarOnLaneSpeedCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles); //
    double NextCarOnLaneDistCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles); 
    double distFromFastestLaneCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles);
    double laneChangeWhenSlowCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles); 
    double getLaneAvgSpeed(const vector<vector<double>> &otherVehicles, unsigned int lane); //
    double getNextCarOnLaneSpeed(const vector<vector<double>> &otherVehicles, unsigned int lane); //
    double getNextCarOnLaneDist(const vector<vector<double>> &otherVehicles, unsigned int lane);
    double getTotalCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles, bool verbose);

    // Select new FSM state and returns the corresponding trajectory
    trajectoryInfo chooseNewState(double lastPathSize, const vector<vector<double>> &otherVehicles, bool verbose);


  private:
    PTG ptg;

    const double TIME_STEP = 0.020; // Every simulator cycle is 20ms long, so the position points are
                                    // separated 20ms apart.
    const unsigned int UPDATE_RATE = 5;

    const double S_POS_STEP = 0.44; // The car will drive on every TIME_STEP (20 ms) 0.44 m if the speed 
                                          // is equal to 22 m/s (49.2126 MPH) so this is used as a standard value
                                          // to generate trajectories.

    const unsigned int TRAJ_LENGTH = 50;   // The standard generated trajectory has 100 points

    const double SPEED_LIMIT = 22.0;        // The speed limit is the end speed value used in order to generate trajectories
                                            // This value is in m/s and is equal to 49.2126 MPH

    const double DESIRED_ACC = 1.0;
    const double DESIRED_JERK = 2.0;
    const double DISTANCE_BUFFER = 30.0;
    const double SEARCH_RANGE = 10.0;

    const double LANE_WIDTH = 4.0;

    const double MAX_ACCEL = 10.0;
    const double MAX_JERK = 10.0;

    const double WEIGHT_AVG_LANE_SPEED = 1500.0;
    const double WEIGHT_NEXT_CAR_ON_LANE_SPEED = 500.0;
    const double WEIGHT_NEXT_CAR_ON_LANE_DIST = 500.0;
    const double WEIGHT_DIST_FASTEST_LANE = 1400.0;
    const double WEIGHT_LANE_CHANGE_WHEN_SLOW = 900.0;

    unsigned int lanes_quantity;

    unsigned int current_cycle; 

    double current_speed_s;
    double current_speed_d;
    double current_acc_s;
    double current_acc_d;
    int current_lane;
    double current_pos_s;

    double target_speed_xy;
    double current_speed_xy;
    double target_acc_xy;
    double current_acc_xy;

    std::map<string, int> lane_direction = {{"PLCL1", -1}, {"PLCR1", 1}, 
                                            {"LCL", -1}, {"LCR", 1},
                                            {"PLCL2", -1}, {"PLCR2", 1},
                                            {"PLCL3", -1}, {"PLCR3", 1},
                                            {"PLCL4", -1}, {"PLCR4", 1},
                                            {"PLCL5", -1}, {"PLCR5", 1},
                                            };

    string current_FSM_state;   

};

typedef double (ego_vehicle::*costFunction_ego) (trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles);

#endif // _EGO_VEHICLE_H_