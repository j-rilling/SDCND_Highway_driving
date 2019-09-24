#include "ego_vehicle.h"

ego_vehicle::ego_vehicle() {
    lanes_quantity = 3;
    current_cycle = 1;
    current_speed_s = 0.0;
    current_speed_d = 0.0;
    current_acc_s = 0.0;
    current_acc_d = 0.0;
    current_lane = 1;
    current_pos_s = 0.0;
    target_speed_xy = SPEED_LIMIT;
    current_speed_xy = 0.0;
    target_acc_xy = 10.0*TIME_STEP;
    current_acc_xy = 0.0*TIME_STEP;
    current_FSM_state = "KL";
}

ego_vehicle::~ego_vehicle() {

}

// This is a test method in order to test the comunication with the simulator
vector<vector<double>> ego_vehicle::getCircularTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints) {
    current_cycle = (current_cycle+1)%UPDATE_RATE;
    double start_x;
    double start_y;
    double start_theta;

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    double path_size = previousXpoints.size();
    if (current_cycle == 1) {
        if (path_size == 0) {
            start_x = x0;
            start_y = y0;
            start_theta = deg2rad(th0);
        }
        else {
            start_x = previousXpoints[path_size -1];
            start_y = previousYpoints[path_size -1];

            double start_x2 = previousXpoints[path_size - 2];
            double start_y2 = previousYpoints[path_size - 2];
            start_theta = atan2(start_y - start_y2, start_x - start_x2); 
            for (unsigned int i = 0; i < path_size; i++) {
                next_x_vals.push_back(previousXpoints[i]);
                next_y_vals.push_back(previousYpoints[i]);
            }
        }

        double dist_inc = 0.5;
        double pos_x = start_x;
        double pos_y = start_y;
        for (unsigned int i = 0; i < 50 - path_size; i++) {
            next_x_vals.push_back(pos_x + dist_inc*cos(start_theta+(i+1)*(pi()/100)));
            next_y_vals.push_back(pos_y + dist_inc*sin(start_theta+(i+1)*(pi()/100)));
            pos_x += dist_inc*cos(start_theta+(i+1)*(pi()/100));
            pos_y += dist_inc*sin(start_theta+(i+1)*(pi()/100));
        }
        return {next_x_vals, next_y_vals};
    } 
    else {
        return {previousXpoints, previousYpoints};
    }
}

vector<vector<double>> ego_vehicle::PTGkeepLineTraj(double x0, double y0, double th0, double v0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY) {
    double start_x, start_y, start_theta, start_s, start_d;

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    unsigned int last_path_size = previousXpoints.size();
    if (last_path_size < 5) {
        start_x = x0;
        start_y = y0;
        start_theta = deg2rad(th0);
    }
    else {
        start_x = previousXpoints[last_path_size - 1];
        start_y = previousYpoints[last_path_size - 1];
        double start_x2 = previousXpoints[last_path_size - 2];
        double start_y2 = previousYpoints[last_path_size - 2];
        start_theta = atan2(start_y - start_y2, start_x - start_x2);
        for (unsigned int i = 0; i < last_path_size; i++) {
            next_x_vals.push_back(previousXpoints[i]);
            next_y_vals.push_back(previousYpoints[i]);
        }
    }

    vector<double> start_sd = getFrenet(start_x, start_y, start_theta, mapsX, mapsY);
    start_s = start_sd[0];
    start_d = start_sd[1];

    vector<double> start_s_vector {start_s, current_speed_s, current_acc_s};
    vector<double> start_d_vector {start_d, current_speed_d, current_acc_d};

    double T = 5.0;        

    double end_acc_s = std::min(current_acc_s + DESIRED_JERK*T, DESIRED_ACC);

    double end_speed_s = std::min(current_speed_s + end_acc_s*T, SPEED_LIMIT);

    double end_s = start_s + end_speed_s*T;
    double end_d = 6.0; // On this method d is not changed.

    vector<double> end_s_vector {end_s, end_speed_s, end_acc_s};
    vector<double> end_d_vector {end_d, 0.0, 0.0};

    std::map<int, vehicle> pred; // Empty, it will be used in the future.

    trajInfo best_trajectory = ptg.getBestTrajectory(start_s_vector, start_d_vector, end_s_vector, 
                                                    end_d_vector, T, pred);

    vector<double> new_s_points;
    vector<double> new_d_points;
    vector<double> sd_t_points;

    double sd_traj_time = TIME_STEP; 

    while (sd_traj_time <= best_trajectory.final_time) {
        double current_s_point = toEquation(best_trajectory.s_coeffs, sd_traj_time);
        double current_d_point = toEquation(best_trajectory.d_coeffs, sd_traj_time);
        sd_t_points.push_back(sd_traj_time);
        new_s_points.push_back(current_s_point);
        new_d_points.push_back(current_d_point);
        sd_traj_time += 10*TIME_STEP;
    }

    vector<vector<double>> new_xy_points;
    new_xy_points = trajFrenetToXY(new_s_points, new_d_points, mapsS, mapsX, mapsY);

    // Here the spline library is used
    tk::spline new_x_spline;
    tk::spline new_y_spline;

    vector<double> last_time_values_for_spline;
    vector<double> last_x_for_spline;
    vector<double> last_y_for_spline;

    for (int i = -1; i >= 0; i++) {
        last_time_values_for_spline.push_back(static_cast<double>(i)*TIME_STEP);
        last_x_for_spline.push_back(previousXpoints[last_path_size + i - 1]);
        last_y_for_spline.push_back(previousYpoints[last_path_size + i - 1]);
    }

    vector<double> time_points_spline;
    time_points_spline.insert(time_points_spline.end(), last_time_values_for_spline.begin(), last_time_values_for_spline.end());
    time_points_spline.insert(time_points_spline.end(), sd_t_points.begin(), sd_t_points.end());

    vector<double> x_points_spline;
    x_points_spline.insert(x_points_spline.end(), last_x_for_spline.begin(), last_x_for_spline.end());
    x_points_spline.insert(x_points_spline.end(), new_xy_points[0].begin(), new_xy_points[0].end());

    vector<double> y_points_spline;
    y_points_spline.insert(y_points_spline.end(), last_y_for_spline.begin(), last_y_for_spline.end());
    y_points_spline.insert(y_points_spline.end(), new_xy_points[1].begin(), new_xy_points[1].end());

    new_x_spline.set_points(time_points_spline, x_points_spline);
    new_y_spline.set_points(time_points_spline, y_points_spline);


    for (int i = 1; i < (TRAJ_LENGTH - last_path_size); i++) {
        next_x_vals.push_back(new_x_spline(TIME_STEP*static_cast<double>(i)));
        next_y_vals.push_back(new_y_spline(TIME_STEP*static_cast<double>(i)));
    }

    vector<double> ds_dt_coeffs = differentiate(best_trajectory.s_coeffs);
    vector<double> d2s_dt2_coeffs = differentiate(ds_dt_coeffs);
    vector<double> dd_dt_coeffs = differentiate(best_trajectory.d_coeffs);
    vector<double> d2d_dt2_coeffs = differentiate(dd_dt_coeffs);

    double last_time_this_traj = static_cast<double>(TRAJ_LENGTH - last_path_size - 1)*TIME_STEP;

    current_speed_s = toEquation(ds_dt_coeffs, last_time_this_traj);
    current_speed_d = toEquation(dd_dt_coeffs, last_time_this_traj);
    current_acc_s = toEquation(d2s_dt2_coeffs, last_time_this_traj);
    current_acc_d = toEquation(d2d_dt2_coeffs, last_time_this_traj);

    return {next_x_vals, next_y_vals};

}

// This method gets the yaw values of a trajectory in XY. These values are needed in order to transform this 
// trajectory in XY to Frenet coordinates. The method was tested getting a lot of points in a circular 
// trajectory generated by "getCircularTraj", so the thetas of this trajectory are continuous between -pi and pi.
// The expected result was obtained on the test.
vector<double> ego_vehicle::getThetasFromXY(const vector<double> &xPoints, const vector<double> &yPoints) {
    unsigned int points_length = xPoints.size();
    vector<double> thetaValues;
    for (unsigned int i = points_length - 1; i > 0; i--) {
        thetaValues.push_back(atan2(yPoints[i] - yPoints[i-1], xPoints[i] - xPoints[i-1]));
    }
    // For the theta of the first pair, the theta of the second pair is used
    thetaValues.push_back(atan2(yPoints[1] - yPoints[0], xPoints[1] - xPoints[0]));
    std::reverse(thetaValues.begin(), thetaValues.end());

    return thetaValues;
}

// This method converts a trajectory in XY coordinates to Frenet coordinates.
vector<vector<double>> ego_vehicle::trajXYToFrenet(const vector<double> &xPoints, const vector<double> &yPoints, 
    const vector<double> &thPoints, const vector<double> &mapsX, const vector<double> &mapsY) {
    vector<double> s_points;
    vector<double> d_points;
    // unsigned int filter_size = 3;
    // unsigned int filter_current_size;
    for (unsigned int i = 0; i < xPoints.size(); i++) {
        vector<double> current_sd_point;
        current_sd_point = getFrenet(xPoints[i], yPoints[i], thPoints[i], mapsX, mapsY);
        s_points.push_back(current_sd_point[0]);
        d_points.push_back(current_sd_point[1]);
    }
    /*
    for (unsigned int i = 0; i < s_points.size(); i++) {
        if (i > s_points.size() - filter_size) {
            filter_current_size = s_points.size() - i;
        }
        else {
            filter_current_size = filter_size;
        }
        double sum_curr_s_point = 0.0;
        double sum_curr_d_point = 0.0;
        for (unsigned int j = i; j < (i + filter_current_size); j++){
            sum_curr_s_point += s_points[j];
            sum_curr_d_point += d_points[j];
        }
        s_points[i] = sum_curr_s_point/filter_current_size;
        d_points[i] = sum_curr_d_point/filter_current_size;
    } */
    return {s_points, d_points};
}

vector<vector<double>> ego_vehicle::trajFrenetToXY(const vector<double> &sPoints, const vector<double> &dPoints,
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY) {
    vector<double> x_points;
    vector<double> y_points;
    unsigned int filter_size = 15;
    unsigned int filter_current_size;
    for (unsigned int i = 0; i < sPoints.size(); i++) {
        vector<double> current_xy_point;
        current_xy_point = getXY(sPoints[i], dPoints[i], mapsS, mapsX, mapsY);
        x_points.push_back(current_xy_point[0]);
        y_points.push_back(current_xy_point[1]);
    }
    /*
    for (unsigned int i = 0; i < x_points.size(); i++) {
        if (i > x_points.size() - filter_size) {
            filter_current_size = x_points.size() - i;
        }
        else {
            filter_current_size = filter_size;
        }
        double sum_curr_x_point = 0.0;
        double sum_curr_y_point = 0.0;
        for (unsigned int j = i; j < (i + filter_current_size); j++){
            sum_curr_x_point += x_points[j];
            sum_curr_y_point += y_points[j];
        }
        x_points[i] = sum_curr_x_point/filter_current_size;
        y_points[i] = sum_curr_y_point/filter_current_size;
    } 
    */
    return {x_points, y_points};
}

// It generates speed values separated UPDATE_RATE*5 (100 ms) from each other
vector<double> ego_vehicle::getVelocity(const vector<double> &posPoints) {
    list<double> speeds_list;
    unsigned int pos_size = posPoints.size();

    for (unsigned int i = 0; i < pos_size - 1; i++) {
        speeds_list.push_back((posPoints[i+1] - posPoints[i])/this->TIME_STEP);
    }
    while (speeds_list.size()%5 != 0) {
        speeds_list.pop_front();
    }
    vector<double> speeds_vector(speeds_list.begin(), speeds_list.end());
    vector<double> avg_speeds_vector;
    for (unsigned int i = 0; i < speeds_list.size(); i+=5) {
        double curr_speed_sum = 0.0;
        for (unsigned int j = i; j < i + 5; j++) {
            curr_speed_sum += speeds_vector[j];
        }
        avg_speeds_vector.push_back(curr_speed_sum/5.0);
    }
    return avg_speeds_vector;
}

vector<double> ego_vehicle::getAcceleration(const vector<double> &velPoints) {
    vector<double> accs;
    unsigned int vel_size = velPoints.size();

    for (unsigned int i = 0; i < vel_size - 1; i++) {
        accs.push_back((velPoints[i+1] - velPoints[i])/this->TIME_STEP);
    }
    return accs;
}

// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
// Later these points are used in order to create a smooth trajectory using spline interpolation
vector<vector<double>> ego_vehicle::SplineTraj(double x0, double y0, double th0, 
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY) {
    unsigned int last_path_size = previousXpoints.size();
    vector<double> points_spline_x;
    vector<double> points_spline_y;

    // Reference x, y, yaw states
    double ref_x = x0;
    double ref_y = y0;
    double ref_yaw = deg2rad(th0);

    // If previous size is almost empty, use the car as starting reference
    if (last_path_size < 2) {
        // Use two points that make the path tangent to the car
        double prev_car_x = x0 - cos(ref_yaw);
        double prev_car_y = y0 - sin(ref_yaw);

        points_spline_x.push_back(prev_car_x);
        points_spline_x.push_back(ref_x);

        points_spline_y.push_back(prev_car_y);
        points_spline_y.push_back(ref_y);
    }
    // Use the previous path's end points as starting reference
    else {
        // Redefine reference state as previous path end point
        ref_x = previousXpoints[last_path_size - 1];
        ref_y = previousYpoints[last_path_size - 1];

        double ref_x_prev = previousXpoints[last_path_size - 2];
        double ref_y_prev = previousYpoints[last_path_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use two points that make the path tangent to the previous path's end point
        points_spline_x.push_back(ref_x_prev);
        points_spline_x.push_back(ref_x);
        
        points_spline_y.push_back(ref_y_prev);
        points_spline_y.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> new_spline_point_1 = getXY(this->current_pos_s + 30,
                                              (2.0+4.0*static_cast<double>(this->current_lane)), mapsS, mapsX, mapsY);
    vector<double> new_spline_point_2 = getXY(this->current_pos_s + 60,
                                              (2.0+4.0*static_cast<double>(this->current_lane)), mapsS, mapsX, mapsY);
    vector<double> new_spline_point_3 = getXY(this->current_pos_s + 90,
                                              (2.0+4.0*static_cast<double>(this->current_lane)), mapsS, mapsX, mapsY);

    points_spline_x.push_back(new_spline_point_1[0]);
    points_spline_x.push_back(new_spline_point_2[0]);
    points_spline_x.push_back(new_spline_point_3[0]);

    points_spline_y.push_back(new_spline_point_1[1]);
    points_spline_y.push_back(new_spline_point_2[1]);
    points_spline_y.push_back(new_spline_point_3[1]);


    // Shift car reference angle to 0 degrees
    for (unsigned int i = 0; i < points_spline_x.size(); i++) {
        double shift_x = points_spline_x[i] - ref_x;
        double shift_y = points_spline_y[i] - ref_y;

        points_spline_x[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
        points_spline_y[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
    }

    // Deletes possible duplicated interpolation points for the spline function
    // in order to avoid errors with the interpolation
    for (unsigned int i = 0; i < points_spline_x.size() - 1; i++) {
        if (points_spline_x[i] == points_spline_x[i+1]) {
            points_spline_x.erase(points_spline_x.begin()+i);
            points_spline_y.erase(points_spline_y.begin()+i);
        }
    }

    // Create a spline
    tk::spline new_x_spline;

    // Set (x,y) points to the spline
    new_x_spline.set_points(points_spline_x, points_spline_y);

    // Define the actual points that will be used by the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with all the previous path points from last time
    for (unsigned int i = 0; i < previousXpoints.size(); i++) {
        next_x_vals.push_back(previousXpoints[i]);
        next_y_vals.push_back(previousYpoints[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = new_x_spline(target_x);
    double target_dist = distance(0,0,target_x, target_y);

    double x_add_on = 0.0;

    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for (unsigned int i = 1; i <= TRAJ_LENGTH - previousXpoints.size(); i++) {
        double N = target_dist/(TIME_STEP*current_speed_xy);
        double x_point = x_add_on + (target_x/N);
        double y_point = new_x_spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back to normal after rotating it earlier
        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return {next_x_vals, next_y_vals};
}

// This method is responsible of the changing on the trajectory. It uses a finite state machine
// to determine what action the ego vehicle should take on the next cycle.
void ego_vehicle::updateTrajectory(const vector<double> &previousXpoints, double s0, 
                            double endPathS, const vector<vector<double>> &otherCars) {
    unsigned int last_path_size = previousXpoints.size();

    if (last_path_size > 0) {
        this->current_pos_s = endPathS;
    }
    else {
        this->current_pos_s = s0;
    }
    
    trajectoryInfo new_trajectory = chooseNewState(last_path_size, otherCars, true);
    this->current_lane = new_trajectory.final_lane;

    if (this->current_acc_xy < new_trajectory.acceleration) {
        this->current_acc_xy += 20.0*TIME_STEP;
    }
    else if (this->current_acc_xy > new_trajectory.acceleration) {
        this->current_acc_xy -= 20.0*TIME_STEP;
    }

    if (this->current_speed_xy < new_trajectory.velocity) {
        this->current_speed_xy += abs(this->current_acc_xy)*TIME_STEP;
    }
    else if (this->current_speed_xy > new_trajectory.velocity) {
        this->current_speed_xy -= abs(this->current_acc_xy)*TIME_STEP;
    }
}

// Returns true if a vehicle is found ahead of the ego vehicle, false otherwise
// The found vehicle is passed as reference through "vehicleFound"
bool ego_vehicle::getVehicleAhead(double lastPathSize, const vector<vector<double>> &otherVehicles, 
                                 unsigned int lane, vector<double> &vehicleFound) {
    bool found_vehicle = false;
    // Min s starts being the expected position of the ego car with a trajectory exactly as long as
    // the last one with current speed and is replaced by the position of cars found at the current
    // lane of the ego car.
    double min_s = this->current_pos_s + DISTANCE_BUFFER*TIME_STEP*this->current_speed_xy;
    // This value corresponds to the s value of the front of the car, since "current_pos_s" actually has the 
    // s value at the end of the already planned trajectory (the green points shown on the simulator)
    double real_pos_car = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;

    double lane_right_border = (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) + (LANE_WIDTH/2.0);
    double lane_left_border =  (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) - (LANE_WIDTH/2.0);

    for (unsigned int i = 0; i < otherVehicles.size(); i++) {
        // If other car is in current lane of ego car
        double other_vehicle_d = otherVehicles[i][6];
        if (other_vehicle_d > lane_left_border && other_vehicle_d < lane_right_border) {
            double other_vehicle_s = otherVehicles[i][5];
            // Check s values greather than of ego car and smaller than min s
            if ((other_vehicle_s > real_pos_car) && (other_vehicle_s < min_s)) {
                found_vehicle = true;
                min_s = other_vehicle_s;
                vehicleFound = otherVehicles[i];
            }
        } 
    }
    return found_vehicle;
}

// Returns true if a vehicle is found behind of the ego vehicle, false otherwise
// The found vehicle is passed as reference through "vehicleFound" 
bool ego_vehicle::getVehicleBehind(const vector<vector<double>> &otherVehicles, 
                                   unsigned int lane, vector<double> &vehicleFound) {                
    bool found_vehicle = false;
    // This value corresponds to the s value of the front of the car, since "current_pos_s" actually has the 
    // s value at the end of the already planned trajectory (the green points shown on the simulator)
    double real_pos_car = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;

    double max_s = real_pos_car - this->SEARCH_RANGE;

    double lane_right_border = (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) + (LANE_WIDTH/2.0);
    double lane_left_border =  (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) - (LANE_WIDTH/2.0);

    for (unsigned int i = 0; i < otherVehicles.size(); i++) {
        // If other car is in current lane of ego car
        double other_vehicle_d = otherVehicles[i][6];
        if (other_vehicle_d > lane_left_border && other_vehicle_d < lane_right_border) {
            double other_vehicle_s = otherVehicles[i][5];
            // Check s values smaller than of ego car and bigger than max s
            if ((other_vehicle_s < this->current_pos_s) && (other_vehicle_s > max_s)) {
                found_vehicle = true;
                max_s = other_vehicle_s;
                vehicleFound = otherVehicles[i];
            }
        }
    }
    return found_vehicle;
}

// Gets the next timestep velocity and acceleration of a given lane. 
// Tries to choose the maximum velocity and acceleration given other vehicle positions and accel/velocity constraints.
vector<double> ego_vehicle::getKinematicsOfLane(double lastPathSize, const vector<vector<double>> &otherVehicles, unsigned int lane) {
    double max_velocity_with_accel_limit = this->MAX_ACCEL + this->current_speed_xy;

    double new_velocity;
    double new_acceleration;
    vector<double> vehicle_ahead;
    vector<double> vehicle_behind;
    // This value corresponds to the s value of the front of the car, since "current_pos_s" actually has the 
    // s value at the end of the already planned trajectory (the green points shown on the simulator)
    double real_pos_car = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;

    if (getVehicleAhead(lastPathSize, otherVehicles, lane, vehicle_ahead)) {
        //std::cout << "Vehicle detected ahead" << std::endl;
        double vehicle_ahead_velocity = sqrt(pow(vehicle_ahead[3],2)+pow(vehicle_ahead[4],2));
        double vehicle_ahead_s = vehicle_ahead[5];
        // The max velocity of the ego vehicle if another vehicle is ahead of it is determined by 
        // the velocity of that detected vehicle plus the distance between the two vehicles minus a distance buffer 
        // That means, that if the distance between the two vehicles is exactly equal to the distance buffer, the ego vehicle 
        // will take the velocity of the other vehicle, if this distance is smaller than the buffer, the velocity will get 
        // smaller and if the distance is bigger than the buffer, the velocity will get bigger. This is subtracted by half the 
        // current acceleration of the ego vehicle in order to compensate the change on the velocity.
        double max_velocity_in_front = (vehicle_ahead_s - real_pos_car - this->DISTANCE_BUFFER) + 
                                            vehicle_ahead_velocity - 0.5*abs(this->current_acc_xy);
        // That calculated velocity gets capped by the maximum possible velocity the vehicle can produce with the maximum
        // acceleration and by the target speed.
        new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_with_accel_limit), this->target_speed_xy);
    } 
    else {
        new_velocity = std::min(max_velocity_with_accel_limit, this->target_speed_xy);
    }

    new_acceleration = new_velocity - this->current_speed_xy;

    return {new_velocity, new_acceleration};
}

// Method for trajectory generation for state "keep lane (KL)"
trajectoryInfo ego_vehicle::keepLaneTraj(double lastPathSize, const vector<vector<double>> &otherVehicles) {
    vector<double> new_vel_acc = getKinematicsOfLane(lastPathSize, otherVehicles, this->current_lane);
    trajectoryInfo new_trajectory;
    new_trajectory.start_lane = this->current_lane;
    new_trajectory.intended_lane = this->current_lane;
    new_trajectory.final_lane = this->current_lane;
    new_trajectory.start_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    new_trajectory.velocity = new_vel_acc[0];
    new_trajectory.end_s = this->current_pos_s + static_cast<double>(TRAJ_LENGTH - lastPathSize)*0.02*new_trajectory.velocity;
    new_trajectory.acceleration = new_vel_acc[1];
    new_trajectory.state = "KL";
    return new_trajectory;
}

// Method for trajectory generation for states "Prepare lane change left (PLCL)" and "Prepare lane change right (PLCR)"
// On this state the ego vehicle takes the speed and acceleration of the lane it wants to change, but that only if this 
// speed is slower than the one of the current lane, so the vehicle in front of the ego vehicle does not get hit.
// Also if a vehicle behind is detected the current speed and acceleration are kept so the vehicle behind don't crash
// with the ego vehicle.
trajectoryInfo ego_vehicle::prepLaneChangeTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles) {
    trajectoryInfo new_trajectory;
    new_trajectory.start_lane = this->current_lane;
    new_trajectory.final_lane = this->current_lane;
    new_trajectory.state = state;
    new_trajectory.start_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;

    // "new_lane" depends on the "state" given to this method, since this method is used for PCLC and PLCR.
    unsigned int new_lane = this->current_lane + this->lane_direction[state];
    new_trajectory.intended_lane = new_lane;

    vector<double> curr_lane_vel_acc = getKinematicsOfLane(lastPathSize, otherVehicles, this->current_lane);
    vector<double> vehicle_behind;
    if (getVehicleBehind(otherVehicles, this->current_lane, vehicle_behind)) {
        // If there is a car behind the ego vehicle, the speed and acceleration are kept so the car behind does not get hit
        // std::cout << "Car behind, kinematics of current lane kept" << std::endl;
        new_trajectory.velocity = curr_lane_vel_acc[0];
        new_trajectory.acceleration = curr_lane_vel_acc[1];

    }
    else {
        vector<double> best_vel_acc;
        vector<double> next_lane_vel_acc = getKinematicsOfLane(lastPathSize, otherVehicles, new_lane);
        // std::cout << "Next lane v: " << next_lane_vel_acc[0] << " , current lane v: " << curr_lane_vel_acc[0] << std::endl;
        if(next_lane_vel_acc[0] < curr_lane_vel_acc[0]) {
            // std::cout << "No car behind, kinematics of lane " << new_lane << "taken" << std::endl;
            best_vel_acc = next_lane_vel_acc;
        }
        else {
            // std::cout << "No car behind, kinematics of current lane kept" << std::endl;
            best_vel_acc = curr_lane_vel_acc;
        }
        new_trajectory.velocity = best_vel_acc[0];
        new_trajectory.acceleration = best_vel_acc[1];
    }
    new_trajectory.end_s = this->current_pos_s + static_cast<double>(50 - lastPathSize)*0.02*new_trajectory.velocity;
    return new_trajectory;
}

trajectoryInfo ego_vehicle::laneChangeTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles) {
    trajectoryInfo new_trajectory;
    new_trajectory.state = state;
    new_trajectory.start_lane = this->current_lane;
    new_trajectory.start_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    unsigned int new_lane = this->current_lane + this->lane_direction[state];
    vector<double> vehicle_ahead_new_lane;
    vector<double> vehicle_behind_new_lane;
    // Checks if a lane change is possible (if there is not a car over there)
    if (getVehicleAhead(lastPathSize, otherVehicles, new_lane, vehicle_ahead_new_lane) || 
        getVehicleBehind(otherVehicles, new_lane, vehicle_behind_new_lane)) {
        //std::cout << "Lane occupied. Stay on current lane" << std::endl;
        vector<double> old_lane_vel_acc = getKinematicsOfLane(lastPathSize, otherVehicles, this->current_lane);
        new_trajectory.velocity = 0.0;
        new_trajectory.acceleration = 0.0;
        new_trajectory.intended_lane = new_lane;
        new_trajectory.final_lane = this->current_lane;
    }
    else {
        //std::cout << "Changing to new lane" << std::endl;
        vector<double> new_lane_vel_acc = getKinematicsOfLane(lastPathSize, otherVehicles, new_lane);
        new_trajectory.velocity = new_lane_vel_acc[0];
        new_trajectory.acceleration = new_lane_vel_acc[1];
        new_trajectory.intended_lane = new_lane;
        new_trajectory.final_lane = new_lane;
    }
    new_trajectory.end_s = this->current_pos_s + static_cast<double>(TRAJ_LENGTH - lastPathSize)*TIME_STEP*new_trajectory.velocity;
    return new_trajectory;   
}

// Selects the correct trajectory generation method based on a given FSM state
trajectoryInfo ego_vehicle::generateStateTraj(string state, double lastPathSize, const vector<vector<double>> &otherVehicles) {
    trajectoryInfo new_trajectory;
    if (state.compare("KL")==0) {
        new_trajectory = keepLaneTraj(lastPathSize, otherVehicles);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        new_trajectory = laneChangeTraj(state, lastPathSize, otherVehicles);
    }
    else if (state.compare("PLCL0") == 0 || state.compare("PLCR0") == 0 || 
             state.compare("PLCL1") == 0 || state.compare("PLCR1") == 0 || 
             state.compare("PLCL2") == 0 || state.compare("PLCR2") == 0 ||
             state.compare("PLCL3") == 0 || state.compare("PLCR3") == 0 ||
             state.compare("PLCL4") == 0 || state.compare("PLCR4") == 0 ||
             state.compare("PLCL5") == 0 || state.compare("PLCR5") == 0 ||
             state.compare("PLCL6") == 0 || state.compare("PLCR6") == 0 ||
             state.compare("PLCL7") == 0 || state.compare("PLCR7") == 0 ||
             state.compare("PLCL8") == 0 || state.compare("PLCR8") == 0 ||
             state.compare("PLCL9") == 0 || state.compare("PLCR9") == 0) {
        new_trajectory = prepLaneChangeTraj(state, lastPathSize, otherVehicles);
    }
    return new_trajectory;
}

// Returns a vector with all the possible next FSM states based on the current state
vector<string> ego_vehicle::possibleNextStates() {
    vector<string> next_states;
    next_states.push_back("KL");
    if (this->current_FSM_state.compare("KL") == 0) {
        if (this->current_lane < 1) {
            next_states.push_back("PLCR0");
        }
        else if (this->current_lane >= (this->lanes_quantity - 1)) {
            next_states.push_back("PLCL0");
        }
        else {
            next_states.push_back("PLCR0");
            next_states.push_back("PLCL0");
        }
    }    
    else if (this->current_FSM_state.compare("PLCL0") == 0) {
        next_states.push_back("PLCL1");
    }
    else if (this->current_FSM_state.compare("PLCL1") == 0) {
        next_states.push_back("PLCL2");
    }
    else if (this->current_FSM_state.compare("PLCL2") == 0) {
        next_states.push_back("PLCL3");
    }
    else if (this->current_FSM_state.compare("PLCL3") == 0) {
        next_states.push_back("PLCL4");
    }
    else if (this->current_FSM_state.compare("PLCL4") == 0) {
        next_states.push_back("PLCL5");
    }
    else if (this->current_FSM_state.compare("PLCL5") == 0) {
        next_states.push_back("PLCL6");
    }
    else if (this->current_FSM_state.compare("PLCL6") == 0) {
        next_states.push_back("PLCL7");
    }
    else if (this->current_FSM_state.compare("PLCL7") == 0) {
        next_states.push_back("PLCL8");
    }
    else if (this->current_FSM_state.compare("PLCL8") == 0) {
        next_states.push_back("PLCL9");
    }
    else if (this->current_FSM_state.compare("PLCL9") == 0) {
        next_states.push_back("PLCL9");
        next_states.push_back("LCL");
    }
    else if (this->current_FSM_state.compare("PLCR0") == 0) {
        next_states.push_back("PLCR1");
    }
    else if (this->current_FSM_state.compare("PLCR1") == 0) {
        next_states.push_back("PLCR2");
    }
    else if (this->current_FSM_state.compare("PLCR2") == 0) {
        next_states.push_back("PLCR3");
    }
    else if (this->current_FSM_state.compare("PLCR3") == 0) {
        next_states.push_back("PLCR4");
    }
    else if (this->current_FSM_state.compare("PLCR4") == 0) {
        next_states.push_back("PLCR5");
    }
    else if (this->current_FSM_state.compare("PLCR5") == 0) {
        next_states.push_back("PLCR6");
    }
    else if (this->current_FSM_state.compare("PLCR6") == 0) {
        next_states.push_back("PLCR7");
    }
    else if (this->current_FSM_state.compare("PLCR7") == 0) {
        next_states.push_back("PLCR8");
    }
    else if (this->current_FSM_state.compare("PLCR8") == 0) {
        next_states.push_back("PLCR9");
    }
    else if (this->current_FSM_state.compare("PLCR9") == 0) {
        next_states.push_back("PLCR9");
        next_states.push_back("LCR");
    }
    return next_states;
}


// Method that determines the average speed of a given lane. For that, it uses the speed of the vehicles 
// until 100 meters ahead and until 10 meters behind the ego vehicle
double ego_vehicle::getLaneAvgSpeed(const vector<vector<double>> &otherVehicles, unsigned int lane) {
    double car_real_pos_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    
    double horizon_s_ahead = car_real_pos_s + 5.0*SEARCH_RANGE;
    double horizon_s_behind = car_real_pos_s;
    unsigned int car_counter = 0;
    double average_speed = 0.0;

    double lane_right_border = (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) + (LANE_WIDTH/2.0);
    double lane_left_border =  (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) - (LANE_WIDTH/2.0);
    
    for (unsigned int i = 0; i < otherVehicles.size(); i++) {
        double other_vehicle_d = otherVehicles[i][6];
        double other_vehicle_s = otherVehicles[i][5];
        double other_vehicle_vel = sqrt(pow(otherVehicles[i][3],2) + pow(otherVehicles[i][4],2));
        if ((other_vehicle_d > lane_left_border) && (other_vehicle_d < lane_right_border)) {
            if ((other_vehicle_s > horizon_s_behind) && (other_vehicle_s < horizon_s_ahead)) {
                average_speed += other_vehicle_vel;
                car_counter++;
            }
        }
    }
    if (car_counter != 0) {
        average_speed = average_speed/static_cast<double>(car_counter);
    }
    else {
        average_speed = this->target_speed_xy;
    }
    return average_speed;
}

// Function that determines the speed of the closest car ahead on the given lane.
double ego_vehicle::getNextCarOnLaneSpeed(const vector<vector<double>> &otherVehicles, unsigned int lane) {
    double car_real_pos_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    
    double min_s_distance = this->current_pos_s + SEARCH_RANGE;

    double lane_right_border = (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) + (LANE_WIDTH/2.0);
    double lane_left_border =  (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) - (LANE_WIDTH/2.0);

    double next_car_speed = -100.0;
    
    for (unsigned int i = 0; i < otherVehicles.size(); i++) {
        double other_vehicle_d = otherVehicles[i][6];
        double other_vehicle_s = otherVehicles[i][5];
        double other_vehicle_vel = sqrt(pow(otherVehicles[i][3],2) + pow(otherVehicles[i][4],2));
        if ((other_vehicle_d > lane_left_border) && (other_vehicle_d < lane_right_border)) {
            if ((other_vehicle_s > car_real_pos_s) && (other_vehicle_s < min_s_distance)) {
                if (other_vehicle_s < min_s_distance) {
                    next_car_speed = other_vehicle_vel;
                    min_s_distance = other_vehicle_s;
                }
            }
        }
    }
    return next_car_speed;
}

// Gets the distance to the next car on a given lane. It is used by the cost function "NextCarOnLaneDistCost"
double ego_vehicle::getNextCarOnLaneDist(const vector<vector<double>> &otherVehicles, unsigned int lane) {
    double car_real_pos_s = this->current_pos_s - static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    
    double min_s_horizon = this->current_pos_s + SEARCH_RANGE;

    double lane_right_border = (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) + (LANE_WIDTH/2.0);
    double lane_left_border =  (LANE_WIDTH/2.0) + LANE_WIDTH*static_cast<double>(lane) - (LANE_WIDTH/2.0);
    
    for (unsigned int i = 0; i < otherVehicles.size(); i++) {
        double other_vehicle_d = otherVehicles[i][6];
        double other_vehicle_s = otherVehicles[i][5];
        if ((other_vehicle_d > lane_left_border) && (other_vehicle_d < lane_right_border)) {
            if ((other_vehicle_s > car_real_pos_s) && (other_vehicle_s < min_s_horizon)) {
                if (other_vehicle_s < min_s_horizon) {
                    min_s_horizon = other_vehicle_s;
                }
            }
        }
    }
    double min_s_distance = min_s_horizon - car_real_pos_s;
    return min_s_distance;
}

// Gets a cost based on the average speed of the intended lane and the final lanes. If the speed of the intended
// and/or the final lane are smaller than the target speed, a higher cost is returned.
double ego_vehicle::avgLaneSpeedCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double speed_intended_lane = getLaneAvgSpeed(otherVehicles, trajectory.intended_lane);
    double speed_final_lane = getLaneAvgSpeed(otherVehicles, trajectory.final_lane);

    double cost = logistic((2.0*this->target_speed_xy - speed_intended_lane - speed_final_lane)/(2.0*this->target_speed_xy));

    return cost;
}

// Gets a cost based on the speed of the closest vehicle ahead on the intended and final lanes. If there is not a car
// close at one of those lanes, their speed is replaced by the target speed. If there is a slow car at the intended or 
// the final lane, the cost will be higher.
double ego_vehicle::NextCarOnLaneSpeedCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double speed_intended_lane = getNextCarOnLaneSpeed(otherVehicles, trajectory.intended_lane);
    if (speed_intended_lane < -50.0) {
        speed_intended_lane = this->target_speed_xy;
    }
    double speed_final_lane = getNextCarOnLaneSpeed(otherVehicles, trajectory.final_lane);
    if (speed_final_lane < -50.0) {
        speed_final_lane = this->target_speed_xy;
    }

    double cost = logistic((2.0*this->target_speed_xy - speed_intended_lane - speed_final_lane)/(2.0*this->target_speed_xy));

    return cost;
}

// Gets a cost based on the distance to the closest vehicle on the intended and final lanes. 
double ego_vehicle::NextCarOnLaneDistCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double dist_next_car_intended_lane = getNextCarOnLaneDist(otherVehicles, trajectory.intended_lane);
    double dist_next_car_final_lane = getNextCarOnLaneDist(otherVehicles, trajectory.final_lane);
    double max_trajectory_s = SEARCH_RANGE + static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;

    double cost = logistic((2.0*max_trajectory_s - dist_next_car_final_lane - dist_next_car_intended_lane)/(2.0*max_trajectory_s));
}

// Gets a cost based on the distance to the fastest lane, so it is easier for the ego vehicle to move 
// to a lane without cars ahead in the case where that lane is 2 lanes away of the current lane. So it works 
// complementing the cost function "avgLaneSpeedCost" which only decides between the current and the next lane.
double ego_vehicle::distFromFastestLaneCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double max_lane_speed = -100.0;
    unsigned int fastest_lane;

    for (unsigned int i = 0; i < this->lanes_quantity; i++) {
        double new_lane_speed = getLaneAvgSpeed(otherVehicles, i);
        if (new_lane_speed > max_lane_speed) {
            max_lane_speed = new_lane_speed;
            fastest_lane = i;
        } 
    }
    double diff_intended_w_fastest_lane = abs(static_cast<int>(fastest_lane - trajectory.intended_lane));
    double diff_final_w_fastest_lane = abs(static_cast<int>(fastest_lane - trajectory.final_lane));
    double total_lane_change_h2 = (pow(diff_final_w_fastest_lane,2) + pow(diff_intended_w_fastest_lane,2));
    double cost = logistic(total_lane_change_h2/(2.0*pow(this->lanes_quantity - 1,2)));
    return cost;
}

// This cost function avoids lane changes when the speed of the car is extremely low 
// (like at the beginning of the simulation for example).
double ego_vehicle::laneChangeWhenSlowCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double diff_intended_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.intended_lane));
    double diff_final_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.final_lane));
    double diff_target_w_current_speed = this->target_speed_xy - this->current_speed_xy;
    double total_lane_change = (diff_final_w_current_lane + diff_final_w_current_lane);
    double cost = logistic(diff_target_w_current_speed*total_lane_change/(this->target_speed_xy*(this->lanes_quantity-1)*2.0));
    return cost;
}

// This cost function avoids lane changes when the cost of staying at the lane or changing is exactly the same.
// It is the cost function with the lowest cost.
double ego_vehicle::laneChangeCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double diff_intended_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.intended_lane));
    double diff_final_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.final_lane));
    double cost = logistic((diff_final_w_current_lane + diff_intended_w_current_lane)/((this->lanes_quantity-1)*2.0));
    return cost;
}

double ego_vehicle::laneChangeWhenNextCarCloseCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles) {
    double diff_intended_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.intended_lane));
    double diff_final_w_current_lane = abs(static_cast<int>(this->current_lane - trajectory.final_lane));
    double next_car_on_own_lane_dist = getNextCarOnLaneDist(otherVehicles, this->current_lane);
    double max_trajectory_s = SEARCH_RANGE + static_cast<double>(TRAJ_LENGTH)*TIME_STEP*this->current_speed_xy;
    double distance_diff = (max_trajectory_s - next_car_on_own_lane_dist);
    double total_lane_change = (diff_final_w_current_lane + diff_final_w_current_lane);
    double cost = logistic((distance_diff*total_lane_change)/(max_trajectory_s*(this->lanes_quantity-1)*2.0));
}

// This method takes all the cost function results multiplied by their weights and sums them up to get the
// total cost.
double ego_vehicle::getTotalCost(trajectoryInfo trajectory, const vector<vector<double>> &otherVehicles, bool verbose) {
    double total_cost = 0.0;

    // Add here other cost functions and their weights
    vector<costFunction_ego> cost_functions {&ego_vehicle::avgLaneSpeedCost, &ego_vehicle::NextCarOnLaneSpeedCost, 
                                             &ego_vehicle::NextCarOnLaneDistCost, &ego_vehicle::distFromFastestLaneCost,
                                             &ego_vehicle::laneChangeWhenSlowCost, &ego_vehicle::laneChangeCost,
                                             &ego_vehicle::laneChangeWhenNextCarCloseCost};
    vector<double> weights {WEIGHT_AVG_LANE_SPEED, WEIGHT_NEXT_CAR_ON_LANE_SPEED, WEIGHT_NEXT_CAR_ON_LANE_DIST, 
                            WEIGHT_DIST_FASTEST_LANE, WEIGHT_LANE_CHANGE_WHEN_SLOW, WEIGHT_LANE_CHANGE_NORMAL, 
                            WEIGHT_LANE_CHANGE_WHEN_CLOSE_NEXT_CAR};
    vector<string> descriptions {"Average speed of lane: ", "Speed of next car on lane: ", "Distance to next car on lane: ",
                                 "Distance to fastest lane: ", "Lane change when slow: ", "Lane change normal: ", 
                                 "Lane change when next vehicle too close: "};

    for (unsigned int i = 0; i < cost_functions.size(); i++) {
        double new_cost = weights[i]*(this->*cost_functions[i])(trajectory, otherVehicles);
        total_cost += new_cost;
        if (verbose == true) {
            std::cout << descriptions[i] << new_cost << std::endl;
        }
    }

    return total_cost;

}

// This method selects the new FSM state from the reachable states from the current FSM state
// The selection is according the state with the smaller cost. The method returns the trajectory
// associated with the state selected and also changes the internal variable "current_FSM_state" 
trajectoryInfo ego_vehicle::chooseNewState(double lastPathSize, const vector<vector<double>> &otherVehicles, bool verbose) {
    // Only considers states which can be reached from current FSM state
    vector<string> possible_next_states = possibleNextStates();
    // This vector has the values of the costs for every next state
    vector<double> state_costs;
    vector<trajectoryInfo> state_trajectories;

    for(unsigned int i = 0; i < possible_next_states.size(); i++) {
        // Gets the trajectory for this state
        trajectoryInfo next_state_trajectory = generateStateTraj(possible_next_states[i], lastPathSize, otherVehicles);

        // Calculates the cost for this trajectory
        double next_state_cost = getTotalCost(next_state_trajectory, otherVehicles, false);

        state_costs.push_back(next_state_cost);
        state_trajectories.push_back(next_state_trajectory);
    }


    // Finds the minimum cost state
    double min_cost = 9999999;
    unsigned int next_state_selected_index;

    for (unsigned int i = 0; i < possible_next_states.size(); i++) {
        if (state_costs[i] < min_cost) {
            next_state_selected_index = i;
            min_cost = state_costs[i];
        }
    }

    
    // Updates the FSM state on the ego vehicle
    this->current_FSM_state = possible_next_states[next_state_selected_index];
    // Gets again the best cost only for debugging
    if (verbose == true) {
        // Prints to the console the new state
        std::cout << "Next state: " << possible_next_states[next_state_selected_index] << std::endl;
        std::cout << "Costs for next state: " << std::endl;
        double best_traj_cost = getTotalCost(state_trajectories[next_state_selected_index], otherVehicles, true);
        std::cout << "TOTAL COST: " << best_traj_cost << std::endl << std::endl;
    }
    return state_trajectories[next_state_selected_index];
}