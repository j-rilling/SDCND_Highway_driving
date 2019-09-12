#ifndef _PTG_H_
#define _PTG_H_

#include <cmath>
#include <vector>
#include <map>
#include "vehicle.h"
#include "helpers.h"

// This class is responsible of generating polynomial trajectories

struct trajInfo {
  std::vector<double> s_coeffs;
  std::vector<double> d_coeffs;
  double final_time;
};

class PTG {
  public:
    PTG();
    ~PTG();

    std::vector<double> JMT (std::vector<double> &start, std::vector<double> &end, double T);

    double nearestApproach(const trajInfo &trajectory, vehicle predictionOfOneVehicle);

    double nearestApproachToAnyVehicle(const trajInfo &trajectory, const std::map <int, vehicle> &predictions);

    double timeDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    double sDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    double dDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    double collisionCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    double bufferDistCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);  

    double goOutRoadCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);  

    double exceedsSpeedLimitCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);

    double efficiencyCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions);      
    

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

};


#endif //PTG_H