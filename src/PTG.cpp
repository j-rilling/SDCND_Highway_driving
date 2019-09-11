#include "PTG.h"

PTG::PTG() {

}

PTG::~PTG() {

}

std::vector<double> PTG::JMT(std::vector<double> &start, std::vector<double> &end, double T) {
      /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  double a0, a1, a2, a3, a4, a5;
   
   a0 = start[0];
   a1 = start[1];
   a2 = start[2]/2;

   double sp = end[0] - (a0 + a1*T + a2*T*T);
   double sv = end[1] - (a1 + 2.0*a2*T);
   double sa = end[2] - 2.0*a2;

   double det = 1/(2.0*pow(T,9.0));

   a3 = det*(20.0*sp*pow(T,6.0) - 8.0*sv*pow(T,7.0) + sa*pow(T,8.0));
   a4 = det*(-30.0*sp*pow(T,5.0) + 14.0*sv*pow(T,6.0) - 2.0*sa*pow(T,7.0));
   a5 = det*(12.0*sp*pow(T,4.0) - 6.0*sv*pow(T,5.0) + sa*pow(T,6.0));

   return {a0, a1, a2, a3, a4, a5};
}

// Penalizes trajectories that span a duration which is longer or 
// shorter than the duration requested.
double PTG::timeDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions) {
   double t = trajectory.final_time;
   return logistic(abs(t-T)/T);
}

// Penalizes trajectories whose s coordinate (and derivatives) 
//  differ from the goal.
double PTG::sDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   double final_time = trajectory.final_time;

   // It gets the coordinates s and d of the target vehicle at the end of the trajectory 
   vehicle target_vehicle = predictions.at(targetVehicleId);
   vector<double> target_vehicle_final_sd = target_vehicle.stateIn(final_time);
   // It adds the delta to the s and d coordinates of the target vehicle, 
   // getting the position desired by the ego vehicle.
   for (unsigned int i = 0; i < target_vehicle_final_sd.size(); i++) {
      target_vehicle_final_sd[i] += delta[i];
   }
   vector<double> target_vehicle_final_s {target_vehicle_final_sd[0], 
                                          target_vehicle_final_sd[1], 
                                          target_vehicle_final_sd[2]};
   // It gets the end s, s' and s'' of the ego vehicle 
   // using the polynomic coefficients saved in "s"
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);
   vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);

   vector<double> ego_vehicle_final_s {toEquation(s_coeffs, final_time), 
                                       toEquation(ds_dt_coeffs, final_time), 
                                       toEquation(d2s_dt2_coeffs, final_time)};
   // It gets the difference between the final s, s' and s'' of the trajectory and the
   // ones based on the position of the target car, converts it to a value between -1 and 1 
   // and sums it to get the cost.
   double cost = 0.0;
   for (unsigned int i = 0; i < ego_vehicle_final_s.size(); i++) {
      double diff = abs(ego_vehicle_final_s[i] - target_vehicle_final_s[i]);
      cost += logistic(diff/this->SIGMA_S[i]);
   }
   return cost;
}

// Penalizes trajectories whose d coordinate (and derivatives) 
//  differ from the goal.
double PTG::dDiffCost(const trajInfo &trajectory, int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   double final_time = trajectory.final_time;

   // It gets the coordinates s and d of the target vehicle at the end of the trajectory 
   vehicle target_vehicle = predictions.at(targetVehicleId);
   vector<double> target_vehicle_final_sd = target_vehicle.stateIn(final_time);
   // It adds the delta to the s and d coordinates of the target vehicle, 
   // getting the position desired by the ego vehicle.
   for (unsigned int i = 0; i < target_vehicle_final_sd.size(); i++) {
      target_vehicle_final_sd[i] += delta[i];
   }
   vector<double> target_vehicle_final_d {target_vehicle_final_sd[3], 
                                          target_vehicle_final_sd[4], 
                                          target_vehicle_final_sd[5]};
   // It gets the end d, d' and d'' of the ego vehicle 
   // using the polynomic coefficients saved in "d"
   vector<double> dd_dt_coeffs = differentiate(d_coeffs);
   vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);

   vector<double> ego_vehicle_final_d {toEquation(d_coeffs, final_time), 
                                       toEquation(dd_dt_coeffs, final_time), 
                                       toEquation(d2d_dt2_coeffs, final_time)};
   // It gets the difference between the final d, d' and d'' of the trajectory and the
   // ones based on the position of the target car, converts it to a value between -1 and 1 
   // and sums it to get the cost.
   double cost = 0.0;
   for (unsigned int i = 0; i < ego_vehicle_final_d.size(); i++) {
      double diff = abs(ego_vehicle_final_d[i] - target_vehicle_final_d[i]);
      cost += logistic(diff/this->SIGMA_D[i]);
   }
   return cost;
}