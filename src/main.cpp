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
  
  vehicle test_target_car = vehicle({0, 10, 0, 0, 0, 0});
  vehicle test_target_car_2 = vehicle({0, 10, 0, 4, 0, 0});
  std::map<int, vehicle> predictions {{0, test_target_car}};
  int test_target_car_id = 0;

  std::vector<double> delta_car {0,0,0,0,0,0};

  vector<vector<double>> goal = ptg.getGoalBasedOnTarget(test_target_car_id, delta_car, time_given, predictions);

  double total_cost = ptg.calculateTotalCost(test_trajectory, goal[0], goal[1], time_given, predictions, true);
  std::cout << "The total cost is: " << total_cost << std::endl;


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
  trajInfo best_trajectory = ptg.getBestTrajectory(start_s, start_d, goal[0], goal[1], time_given, predictions);
}


int main() {
  tests();
  return 0;
}

