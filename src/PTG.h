#ifndef _PTG_H_
#define _PTG_H_

#include <cmath>
#include <vector>
#include <map>
#include <random>
#include <chrono>
#include <iostream>
#include <string>
#include "vehicle.h"
#include "helpers.h"

// PTG means Polynomial trajectory generator

struct trajInfo {
  std::vector<double> s_coeffs;
  std::vector<double> d_coeffs;
  double final_time;
};

class PTG {
  public:
    PTG();
    ~PTG();

    std::vector<double> JMT (const std::vector<double> &start, const std::vector<double> &end, double T);

    double nearestApproach(const trajInfo &trajectory, vehicle predictionOfOneVehicle);

    double nearestApproachToAnyVehicle(const trajInfo &trajectory, const std::map <int, vehicle> &predictions);

    double timeDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double sDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double dDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double collisionCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double bufferDistCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double goOutRoadCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions); 

    double exceedsSpeedLimitCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double efficiencyCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double totalAccelSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double totalAccelDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

    double maxAccelSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);   

    double maxAccelDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions); 

    double totalJerkSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions); 
    
    double totalJerkDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);
  
    double maxJerkSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions); 
    
    double maxJerkDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions); 
    
    double calculateTotalCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions, bool verbose=false);

    vector<vector<double>> perturbGoal(vector<double> goalS, vector<double> goalD);

    vector<vector<double>> getGoalBasedOnTarget(int targetVehicleId, 
      const vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    /*
    Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that 
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at 
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal 
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a 
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method 
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/ 
     this trajectory. */
    trajInfo getBestTrajectory(const vector<double> &startS, const vector<double> &startD, 
      const vector<double> &goalS, const vector<double> &goalD, double T, 
      const std::map<int, vehicle> &predictions);


  private:
    const double N_SAMPLES = 10;
    const double SIGMA_S[3] = {10.0, 4.0, 2.0};
    const double SIGMA_D[3] = {1.0, 1.0, 1.0};
    const double SIGMA_T = 2.0;
    
    const double MAX_JERK = 10;     // m/(s^3)
    const double MAX_ACCEL = 10;    // m/(s^2)

    const double EXPECTED_JERK_IN_ONE_SEC = 2; // m/(s^2)
    const double EXPECTED_ACC_IN_ONE_SEC = 1;  // m/s

    const double SPEED_LIMIT = 22.352;    // m/s (22.352 m/s = 50 mph)
    const double VEHICLE_RADIUS = 1.5;  // model vehicle as circle to simplify collision detection (change later)

    const double ROAD_D[2] = {0.0, 12.0}; // Change later to {0.0, 12.0}

    const double WEIGHT_TIME_DIFF = 70.0;
    const double WEIGHT_S_DIFF = 200.0;
    const double WEIGHT_D_DIFF = 250.0;
    const double WEIGHT_COLLISION = 500.0;
    const double WEIGHT_BUFFER = 400.0;
    const double WEIGHT_GO_OUT_ROAD = 300.0;
    const double WEIGHT_EXCEED_SPEED_LIMIT = 400.0;
    const double WEIGHT_EFFICIENCY = 130.0;
    const double WEIGHT_TOTAL_ACCEL_S = 130.0;
    const double WEIGHT_TOTAL_ACCEL_D = 50.0;
    const double WEIGHT_MAX_ACCEL_S = 120.0;
    const double WEIGHT_MAX_ACCEL_D = 40.0;
    const double WEIGHT_TOTAL_JERK_S = 350.0;
    const double WEIGHT_TOTAL_JERK_D = 370.0;
    const double WEIGHT_MAX_JERK_S = 340.0;
    const double WEIGHT_MAX_JERK_D = 350.0;
};

typedef double (PTG::*costFunction) (const trajInfo &trajectory, const vector<double> &goalS, 
  const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions);

#endif //PTG_H