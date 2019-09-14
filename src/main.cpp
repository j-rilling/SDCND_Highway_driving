#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "PTG.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main_project() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

void tests() {

  // TEST: JMT Method of PTG class
  std::vector<double> start_s {10, 10, 0};
  std::vector<double> end_s {30, 5, 10};
  std::vector<double> start_d {4, 0, 0};
  std::vector<double> end_d {5, 0, 10};

  PTG ptg = PTG();
  double time_given = 5.0;
  std::vector<double> coeffs_s = ptg.JMT(start_s, end_s, time_given);
  std::vector<double> coeffs_d = ptg.JMT(start_d, end_d, time_given);
  double time_real_traj = 5.0;

  trajInfo test_trajectory {coeffs_s, coeffs_d, time_real_traj};

  std::cout << std::endl << "TEST: JMT Method of PTG class" << std::endl;
  std::cout << "Trajectory for s " << std::endl;
  std::cout << "a0: " << coeffs_s[0] << std::endl;
  std::cout << "a1: " << coeffs_s[1] << std::endl;
  std::cout << "a2: " << coeffs_s[2] << std::endl;
  std::cout << "a3: " << coeffs_s[3] << std::endl;
  std::cout << "a4: " << coeffs_s[4] << std::endl;
  std::cout << "a5: " << coeffs_s[5] << std::endl;
  std::cout << std::endl;

  std::cout << "Trajectory for d " << std::endl;
  std::cout << "a0: " << coeffs_d[0] << std::endl;
  std::cout << "a1: " << coeffs_d[1] << std::endl;
  std::cout << "a2: " << coeffs_d[2] << std::endl;
  std::cout << "a3: " << coeffs_d[3] << std::endl;
  std::cout << "a4: " << coeffs_d[4] << std::endl;
  std::cout << "a5: " << coeffs_d[5] << std::endl;
  std::cout << std::endl;
  
  // Test time difference cost function
  vehicle test_target_car = vehicle({0, 10, 0, 0, 0, 0});
  vehicle test_target_car_2 = vehicle({0, 10, 0, 4, 0, 0});

  std::map<int, vehicle> predictions {{0, test_target_car}};
  int test_target_car_id = 0;

  std::vector<double> delta_car {0,0,0,0,0,0};

  double time_cost = ptg.timeDiffCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);

    std::cout << std::endl << "TEST: Cost functions" << std::endl;
  std::cout << "Cost for time difference between " << time_given << "[s] and " << time_real_traj << "[s]: " << time_cost << std::endl;

  // Test s diff cost function
  double s_diff_cost = ptg.sDiffCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);

  std::vector<double> test_target_car_sd = test_target_car.stateIn(time_real_traj);
  std::vector<double> coeffs_ds_dt = differentiate(coeffs_s);
  std::vector<double> coeffs_d2s_dt2 = differentiate(coeffs_ds_dt);
  
  std::cout << "Cost for s difference between (" << test_target_car_sd[0] << "," << 
                                                    test_target_car_sd[1] << "," << 
                                                    test_target_car_sd[2] << ") (target car) and (" << 
                                                    toEquation(coeffs_s, time_real_traj) << "," << 
                                                    toEquation(coeffs_ds_dt, time_real_traj) << "," << 
                                                    toEquation(coeffs_d2s_dt2, time_real_traj) << 
                                                    ") (ego car) is: " << s_diff_cost << std::endl;

  // Test d diff cost function
  double d_diff_cost = ptg.dDiffCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::vector<double> coeffs_dd_dt = differentiate(coeffs_d);
  std::vector<double> coeffs_d2d_dt2 = differentiate(coeffs_dd_dt);

  std::cout << "Cost for d difference between (" << test_target_car_sd[3] << "," << 
                                                    test_target_car_sd[4] << "," << 
                                                    test_target_car_sd[5] << ") (target car) and (" << 
                                                    toEquation(coeffs_d, time_real_traj) << "," << 
                                                    toEquation(coeffs_dd_dt, time_real_traj) << "," << 
                                                    toEquation(coeffs_d2d_dt2, time_real_traj) << 
                                                    ") (ego car) is: " << d_diff_cost << std::endl;

  // Test of "nearestApproach" method
  double closest_distance_between_cars = ptg.nearestApproach(test_trajectory, test_target_car);
  std::cout << "The nearest approach between both cars is: " << closest_distance_between_cars << std::endl;

  // Test of "nearestApproachToAnyVehicle" method
  double closest_distance_between_all_cars = ptg.nearestApproachToAnyVehicle(test_trajectory, predictions);
  std::cout << "The nearest approach between all cars is: " << closest_distance_between_all_cars << std::endl;

  // Test of "collisionCost" method
  double collision_cost = ptg.collisionCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The collision cost is: " << collision_cost << std::endl;

  // Test of "bufferDistCost" method
  double buffer_dist_cost = ptg.bufferDistCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The buffer distance cost is: " << buffer_dist_cost << std::endl;

  // Test of "goOutRoadCost" method
  double go_out_road_cost = ptg.goOutRoadCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The go out road cost is: " << go_out_road_cost << std::endl;

  // Test of "exceedsSpeedLimitCost" method
  double exceeds_speed_limit_cost = ptg.exceedsSpeedLimitCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The exceed speed limit cost is: " << exceeds_speed_limit_cost << std::endl;

  // Test of "efficiencyCost" method
  double efficiency_cost = ptg.efficiencyCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The efficiency cost is: " << efficiency_cost << std::endl;

  // Test of "totalAccelSCost" method
  double total_acc_s_cost = ptg.totalAccelSCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The total acceleration cost (s) is: " << total_acc_s_cost << std::endl;

  // Test of "totalAccelDCost" method
  double total_acc_d_cost = ptg.totalAccelDCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The total acceleration cost (d) is: " << total_acc_d_cost << std::endl;

  // Test of "maxAccelSCost" method
  double max_acc_s_cost = ptg.maxAccelSCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The max acceleration cost (s) is: " << max_acc_s_cost << std::endl;

  // Test of "maxAccelDCost" method
  double max_acc_d_cost = ptg.maxAccelDCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The max acceleration cost (d) is: " << max_acc_d_cost << std::endl;

  // Tesf of "totalJerkSCost" method
  double total_jerk_s_cost = ptg.totalJerkSCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The total jerk cost (s) is: " << total_jerk_s_cost << std::endl;

  // Tesf of "totalJerkDCost" method
  double total_jerk_d_cost = ptg.totalJerkDCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The total jerk cost (d) is: " << total_jerk_d_cost << std::endl;

  // Tesf of "maxJerkSCost" method
  double max_jerk_s_cost = ptg.maxJerkSCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The max jerk cost (s) is: " << max_jerk_s_cost << std::endl;

  // Tesf of "maxJerkDCost" method
  double max_jerk_d_cost = ptg.maxJerkDCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions);
  std::cout << "The max jerk cost (d) is: " << max_jerk_d_cost << std::endl;

  // Test of "calculateTotalCost" method
  double total_cost = ptg.calculateTotalCost(test_trajectory, test_target_car_id, delta_car, time_given, predictions, true);
  std::cout << "The total cost is : " << total_cost << std::endl;

  // Test of "perturbGoal" method
  std::vector<std::vector<double>> perturbed_end = ptg.perturbGoal(end_s, end_d);
  std::vector<double> perturbed_end_s = perturbed_end[0];
  std::vector<double> perturbed_end_d = perturbed_end[1];

  std::cout << std::endl << "TEST: PERTURB GOAL" << std::endl;
  std::cout << "The pertubed goal (s) is: (" << perturbed_end_s[0] << "), (" <<
                                                perturbed_end_s[1] << "), (" <<
                                                perturbed_end_s[2] << ")" << std::endl;

  std::cout << "The pertubed goal (d) is: (" << perturbed_end_d[0] << "), (" <<
                                                perturbed_end_d[1] << "), (" <<
                                                perturbed_end_d[2] << ")" << std::endl;
  
  // Test of "best trajectory" method
  trajInfo best_trajectory = ptg.getBestFollowTrajectory(start_s, start_d, test_target_car_id, delta_car, time_given, predictions);
}


int main() {
  tests();
  return 0;
}

