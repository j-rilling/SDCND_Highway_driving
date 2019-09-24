#include "PTG.h"

PTG::PTG() {

}

PTG::~PTG() {

}

std::vector<double> PTG::JMT(const vector<double> &start, const vector<double> &end, double T) {
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

// Calculates the closest distance to any vehicle during a trajectory.
double PTG::nearestApproach(const trajInfo &trajectory, vehicle predictionOfOneVehicle) {
   double closest_dist = 999999;
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> d_coeffs = trajectory.d_coeffs;
   double time_end_trajectory = trajectory.final_time;

   for (unsigned int i = 0; i < 100; i++) {
      double t = (static_cast<double>(i)/100.0) * time_end_trajectory;
      double curr_s = toEquation(s_coeffs, t);
      double curr_d = toEquation(d_coeffs, t);
      double targ_s = predictionOfOneVehicle.stateIn(t)[0];
      double targ_d = predictionOfOneVehicle.stateIn(t)[3];
      double dist_ego_curr = distance(curr_s,curr_d, targ_s, targ_d);
      if (dist_ego_curr < closest_dist) {
         closest_dist = dist_ego_curr;
      }
   }
   return closest_dist;
}

// Calculates the closest distance to any vehicle during a trajectory.
double PTG::nearestApproachToAnyVehicle(const trajInfo &trajectory, const std::map <int, vehicle> &predictions) {
   double closest_dist = 999999;
   for (std::map<int, vehicle>::const_iterator it = predictions.begin(); it != predictions.end(); it++) {
      double curr_vehicle_distance = this->nearestApproach(trajectory, it->second);
      if (curr_vehicle_distance < closest_dist) {
         closest_dist = curr_vehicle_distance;
      }
   }
   return closest_dist;
}

// Penalizes trajectories that span a duration which is longer or 
// shorter than the duration requested.
double PTG::timeDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   double t = trajectory.final_time;
   return logistic(abs(t-T)/T);
}

// Penalizes trajectories whose s coordinate (and derivatives) 
//  differ from the goal.
double PTG::sDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   double final_time = trajectory.final_time;

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
      double diff = abs(ego_vehicle_final_s[i] - goalS[i]);
      cost += logistic(diff/this->SIGMA_S[i]);
   }
   return cost;
}

// Penalizes trajectories whose d coordinate (and derivatives) 
//  differ from the goal.
double PTG::dDiffCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   double final_time = trajectory.final_time;

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
      double diff = abs(ego_vehicle_final_d[i] - goalD[i]);
      cost += logistic(diff/this->SIGMA_D[i]);
   }
   return cost;
}

// Binary cost function which penalizes collisions.
double PTG::collisionCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   double nearest_approach = nearestApproachToAnyVehicle(trajectory, predictions);
   if (nearest_approach < 2.0*this->VEHICLE_RADIUS) {
      return 1.0;
   }
   else {
      return 0.0;
   }
}

// Penalizes getting close to other vehicles.
double PTG::bufferDistCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   double nearest_approach = nearestApproachToAnyVehicle(trajectory, predictions);
   return logistic(2.0*this->VEHICLE_RADIUS / nearest_approach);
}

// Penalizes getting out of the road. 
// It calculates the lateral distance the car was out of the road using 100 different 
// points of the trajectory equally separated. Sums up all the distances and returns 
// the logistic function of this sum divided by 100.
double PTG::goOutRoadCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_distance_out_road = 0.0;

   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double current_d = toEquation(d_coeffs, t);
      if ((current_d - this->VEHICLE_RADIUS) < this->ROAD_D[0]) {
         total_distance_out_road += (this->ROAD_D[0] - (current_d - this->VEHICLE_RADIUS));
      }
      else if ((current_d + this->VEHICLE_RADIUS) > this->ROAD_D[1]) {
         total_distance_out_road += ((current_d + this->VEHICLE_RADIUS) - this->ROAD_D[1]);
      }
   }
   return logistic(total_distance_out_road/100.0);
}


// Penalizes exceeding the speed limit.
double PTG::exceedsSpeedLimitCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);   
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_speed_exceeded = 0.0;

   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double current_speed_s = toEquation(ds_dt_coeffs, t);
      if (current_speed_s - this->SPEED_LIMIT > 0.0) {
         total_speed_exceeded += (current_speed_s - this->SPEED_LIMIT);
      }
   }
   return logistic(total_speed_exceeded/100.0);
}

// Rewards average speeds higher than the ones of "target vehicle"
// This means, the cost function gives a cost bigger than 1 if the 
// average speed of the ego vehicle is bigger than the average speed
// of the target vehicle plus a delta. In the contrary case, it gives 
// costs lower than 1. 
// An example for clarification: If delta[1] = -10, that means that
// the desired speed is 10 m/s lower than the speed of the target car.
double PTG::efficiencyCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> dt_ds_coeffs = differentiate(s_coeffs);
   double final_time = trajectory.final_time;
   double avg_ego_speed_s = toEquation(dt_ds_coeffs, final_time)/final_time;
   double avg_target_speed_s = goalS[1]/final_time;
   return logistic(2*(avg_target_speed_s - avg_ego_speed_s)/avg_ego_speed_s);
}

// It penalizes trajectories which have average acceleration higher than the expected 
// acceleration in one second. High acceleration is directly related with the fuel 
// efficiency of the car, so it is also desired to reduce the total acceleration, 
// but is not that important as minimizing jerk.
double PTG::totalAccelSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);
   vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_acc_s = 0.0;
   for (unsigned int i = 0; i < 100; i++){
      double t = dt*static_cast<double>(i);
      total_acc_s += abs(dt*toEquation(d2s_dt2_coeffs,t));
   }
   double avg_acc_s = total_acc_s/final_time;
   return logistic(avg_acc_s/this->EXPECTED_ACC_IN_ONE_SEC);
}

// It penalizes trajectories which have average acceleration higher than the expected 
// acceleration in one second. High acceleration is directly related with the fuel 
// efficiency of the car, so it is also desired to reduce the total acceleration, 
// but is not that important as minimizing jerk.
double PTG::totalAccelDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   vector<double> dd_dt_coeffs = differentiate(d_coeffs);
   vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_acc_d = 0.0;
   for (unsigned int i = 0; i < 100; i++){
      double t = dt*static_cast<double>(i);
      total_acc_d += abs(dt*toEquation(d2d_dt2_coeffs,t));
   }
   double avg_acc_d = total_acc_d/final_time;
   return logistic(avg_acc_d/this->EXPECTED_ACC_IN_ONE_SEC);
}

// It penalizes trajectories which reaches at some point the maximum acceleration the car can produce
// This cost function needs to be weighted accordingly in order to not break the car
double PTG::maxAccelSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);
   vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double max_acc_s = 0.0;
   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double curr_acc_s = toEquation(d2s_dt2_coeffs, t);
      if (curr_acc_s > max_acc_s) {
         max_acc_s = curr_acc_s;
      }
   }
   if (abs(max_acc_s) > this->MAX_ACCEL) {
      return 1;
   }
   else {
      return 0;
   }
}


// It penalizes trajectories which reaches at some point the maximum lateral acceleration the car can produce
// This cost function needs to be weighted accordingly in order to not break the car
double PTG::maxAccelDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   vector<double> dd_dt_coeffs = differentiate(d_coeffs);
   vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double max_acc_d = 0.0;
   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double curr_acc_d = toEquation(d2d_dt2_coeffs, t);
      if (curr_acc_d > max_acc_d) {
         max_acc_d = curr_acc_d;
      }
   }
   if (abs(max_acc_d) > this->MAX_ACCEL) {
      return 1;
   }
   else {
      return 0;
   }
}

// It penalizes trajectories which have average jerk higher than the expected 
// jerk in one second. High jerk is uncomfortable for humans, so this cost function
// needs to have a very high cost.
double PTG::totalJerkSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);
   vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);
   vector<double> d3s_dt3_coeffs = differentiate(d2s_dt2_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_jerk_s = 0.0;;
   for (unsigned int i = 0; i < 100; i++){
      double t = dt*static_cast<double>(i);
      total_jerk_s += abs(dt*toEquation(d3s_dt3_coeffs,t));
   }
   double avg_jerk_s = total_jerk_s/final_time;
   return logistic(avg_jerk_s/this->EXPECTED_JERK_IN_ONE_SEC);
}

// It penalizes trajectories which have average jerk higher than the expected 
// jerk in one second. High jerk is uncomfortable for humans, so this cost function
// needs to have a very high cost.
double PTG::totalJerkDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   vector<double> dd_dt_coeffs = differentiate(d_coeffs);
   vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);
   vector<double> d3d_dt3_coeffs = differentiate(d2d_dt2_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double total_jerk_d = 0.0;;
   for (unsigned int i = 0; i < 100; i++){
      double t = dt*static_cast<double>(i);
      total_jerk_d += abs(dt*toEquation(d3d_dt3_coeffs,t));
   }
   double avg_jerk_d = total_jerk_d/final_time;
   return logistic(avg_jerk_d/this->EXPECTED_JERK_IN_ONE_SEC);
}

// It penalizes trajectories that reach at some point the maximum jerk the car can produce
// This cost function needs to be weighted accordingly in order to not break the car
double PTG::maxJerkSCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> s_coeffs = trajectory.s_coeffs;
   vector<double> ds_dt_coeffs = differentiate(s_coeffs);
   vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);
   vector<double> d3s_dt3_coeffs = differentiate(d2s_dt2_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double max_jerk_s = 0.0;
   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double curr_jerk_s = toEquation(d2s_dt2_coeffs, t);
      if (curr_jerk_s > max_jerk_s) {
         max_jerk_s = curr_jerk_s;
      }
   }
   if (abs(max_jerk_s) > this->MAX_JERK) {
      return 1;
   }
   else {
      return 0;
   }
}

// It penalizes trajectories that reach at some point the maximum jerk the car can produce
// This cost function needs to be weighted accordingly in order to not break the car
double PTG::maxJerkDCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions) {
   vector<double> d_coeffs = trajectory.d_coeffs;
   vector<double> dd_dt_coeffs = differentiate(d_coeffs);
   vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);
   vector<double> d3d_dt3_coeffs = differentiate(d2d_dt2_coeffs);
   double final_time = trajectory.final_time;
   double dt = final_time/100.0;
   double max_jerk_d = 0.0;
   for (unsigned int i = 0; i < 100; i++) {
      double t = dt*static_cast<double>(i);
      double curr_jerk_d = toEquation(d2d_dt2_coeffs, t);
      if (curr_jerk_d > max_jerk_d) {
         max_jerk_d = curr_jerk_d;
      }
   }
   if (abs(max_jerk_d) > this->MAX_JERK) {
      return 1;
   }
   else {
      return 0;
   }
}

// It calculates the total cost using all the cost functions and their weights.
double PTG::calculateTotalCost(const trajInfo &trajectory, const vector<double> &goalS, 
      const vector<double> &goalD, double T, const std::map<int, vehicle> &predictions, bool verbose) {
   vector<costFunction> cfList {&PTG::timeDiffCost, &PTG::sDiffCost, &PTG::dDiffCost, &PTG::collisionCost, &PTG::bufferDistCost, 
                            &PTG::goOutRoadCost, &PTG::exceedsSpeedLimitCost, &PTG::efficiencyCost, &PTG::totalAccelSCost, 
                            &PTG::totalAccelDCost, &PTG::maxAccelSCost, &PTG::maxAccelDCost, &PTG::totalJerkSCost,
                            &PTG::totalJerkDCost, &PTG::maxJerkSCost, &PTG::maxJerkDCost};
   vector<double> weights  {WEIGHT_TIME_DIFF, WEIGHT_S_DIFF, WEIGHT_D_DIFF, WEIGHT_COLLISION, WEIGHT_BUFFER,
                       WEIGHT_GO_OUT_ROAD, WEIGHT_EXCEED_SPEED_LIMIT, WEIGHT_EFFICIENCY, WEIGHT_TOTAL_ACCEL_S,
                       WEIGHT_TOTAL_ACCEL_D, WEIGHT_MAX_ACCEL_S, WEIGHT_MAX_ACCEL_D, WEIGHT_TOTAL_JERK_S,
                       WEIGHT_TOTAL_JERK_D, WEIGHT_MAX_JERK_S, WEIGHT_MAX_JERK_D};
   vector<std::string> fNames = {"time difference ", "s difference ", "d difference ", "collision ", "buffer distance ",
                                 "go out of road ", "exceed speed limit ", "efficiency ", "total s acceleration ", 
                                 "total d acceleration ", "max s acceleration ", "max d acceleration ", "total s jerk ",
                                 "total d jerk ", "max s jerk ", "max d jerk "};

   double total_cost = 0.0;
   for (unsigned int i = 0; i < cfList.size(); i++) {
      double new_cost = weights[i]*(this->*cfList[i])(trajectory, goalS, goalD, T, predictions);
      if (verbose) {
         std::cout << "Cost of " << fNames[i] << "is: " << new_cost << std::endl;
      }
      total_cost += new_cost;

   }
   return total_cost;
}

// Returns a "perturbed" version of the goal.
vector<vector<double>> PTG::perturbGoal(vector<double> goalS, vector<double> goalD) {
   std::default_random_engine random_gen;
   random_gen.seed(std::chrono::system_clock::now().time_since_epoch().count());
   std::normal_distribution<double> s_pos_noise(goalS[0], this->SIGMA_S[0]);
   std::normal_distribution<double> s_vel_noise(goalS[1], this->SIGMA_S[1]);
   std::normal_distribution<double> s_acc_noise(goalS[2], this->SIGMA_S[2]);
   std::normal_distribution<double> d_pos_noise(goalD[0], this->SIGMA_D[0]);
   std::normal_distribution<double> d_vel_noise(goalD[1], this->SIGMA_D[1]);
   std::normal_distribution<double> d_acc_noise(goalD[2], this->SIGMA_D[2]);

   vector<double> new_s_goal {s_pos_noise(random_gen), s_vel_noise(random_gen), s_acc_noise(random_gen)};
   vector<double> new_d_goal {d_pos_noise(random_gen), d_vel_noise(random_gen), d_acc_noise(random_gen)};

   vector<vector<double>> new_goal {new_s_goal, new_d_goal};

   return new_goal;
}

// Returns the desired goal based on a target vehicle and a delta. Used in order to be able to generate 
// trajectories without a target car, where some cost functions are discarded.
vector<vector<double>> PTG::getGoalBasedOnTarget(int targetVehicleId, 
      const std::vector<double> &delta, double T, const std::map<int, vehicle> &predictions) {
   vehicle target_vehicle = predictions.at(targetVehicleId);
   double goal_s_pos = target_vehicle.stateIn(T)[0] + delta[0];
   double goal_s_vel = target_vehicle.stateIn(T)[1] + delta[1];
   double goal_s_acc = target_vehicle.stateIn(T)[2] + delta[2];
   double goal_d_pos = target_vehicle.stateIn(T)[3] + delta[3];
   double goal_d_vel = target_vehicle.stateIn(T)[4] + delta[4];
   double goal_d_acc = target_vehicle.stateIn(T)[5] + delta[5];

   vector<double> goal_s {goal_s_pos, goal_s_vel, goal_s_acc};
   vector<double> goal_p {goal_d_pos, goal_d_vel, goal_d_acc};

   return {goal_s, goal_p};
}

trajInfo PTG::getBestTrajectory(const vector<double> &startS, const vector<double> &startD, 
      const vector<double> &goalS, const vector<double> &goalD, double T, 
      const std::map<int, vehicle> &predictions) {

   vector<vector<double>> all_goals_s;
   vector<vector<double>> all_goals_d;
   vector<double> all_times;
   double timestep = 0.5;
   double t = T - 4*timestep;
   // Gets goals arround 4 time steps before and after the desired time
   while (t <= T + 4*timestep) {
      // It gets N_SAMPLES different random possible positions of the target car at time t
      for (unsigned int i = 0; i < this->N_SAMPLES; i++) {
         vector<vector<double>> perturbed_goal = perturbGoal(goalS, goalD);
         all_goals_s.push_back(perturbed_goal[0]);
         all_goals_d.push_back(perturbed_goal[1]);
         all_times.push_back(t);
      }
      t += timestep;
   }

   // Vector to save the desired trajectories
   vector<trajInfo> trajectories;

   // It gets trayectories for all the randomized last positions of the target car
   for (unsigned int i = 0; i < all_goals_d.size(); i++) {
      trajInfo curr_trajectory;
      curr_trajectory.s_coeffs = JMT(startS, all_goals_s[i], all_times[i]);
      curr_trajectory.d_coeffs = JMT(startD, all_goals_d[i], all_times[i]);
      curr_trajectory.final_time = all_times[i];
      // It appends all the possible trajectories (its polynomic coefficients actually) in "trajectories"
      trajectories.push_back(curr_trajectory);
   }

   // It gets the best trajectory by getting the trajectory which yields the minimum cost 
   // using all the cost functions and their weights 
   trajInfo best_trajectory;
   double min_cost = 999999;
   for (unsigned int i = 0; i < trajectories.size(); i++) {
      double traj_cost = calculateTotalCost(trajectories[i], goalS, goalD, T, predictions, false);
      if (traj_cost < min_cost) {
         min_cost = traj_cost;
         best_trajectory = trajectories[i];
      }
   }

   // It calculates the weight of the best trajectory again and prints it
   calculateTotalCost(best_trajectory, goalS, goalD, T, predictions, true);
   return best_trajectory;
}

