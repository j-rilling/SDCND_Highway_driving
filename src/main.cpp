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
#include "ego_vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void writeToCSV (double first, double second, double third, double fourth) {
  std::ofstream outputFile;
  std::string filename = "log.csv";
  outputFile.open(filename.c_str(), std::ios::out | std::ios::app);
  if (outputFile.fail()){
    std::cout << "Log failed to open" << std::endl;
  }
  outputFile << first << " " << second << " " << third << " " << fourth << std::endl;
  outputFile.close();
}

vector<vector<double>> readFromCSV () {
  vector <double> x_values;
  vector <double> y_values;

  std::ifstream inputFile;
  std::string filename = "log.csv";
  inputFile.open(filename.c_str(), std::ifstream::in);
  string line;
  while (getline(inputFile, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    iss >> x;
    iss >> y;
    x_values.push_back(x);
    y_values.push_back(y);
  }
  inputFile.close();
  return {x_values, y_values};
}

int main() {
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

  // Ego vehicle instance
  ego_vehicle ego_v = ego_vehicle();

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
               &map_waypoints_dx,&map_waypoints_dy, &ego_v]
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

          writeToCSV(car_x, car_y, car_yaw, car_speed);



          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<vector<double>> next_xy_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //next_xy_vals = ego_v.PTGkeepLineTraj(car_x, car_y, car_yaw, car_speed, previous_path_x, previous_path_y,
          //                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ego_v.updateTrajectory(previous_path_x, car_s, end_path_s, sensor_fusion);
          next_xy_vals = ego_v.GetNewPosPoints(car_x, car_y, car_yaw, previous_path_x, previous_path_y, 
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);

          msgJson["next_x"] = next_xy_vals[0];
          msgJson["next_y"] = next_xy_vals[1];

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

