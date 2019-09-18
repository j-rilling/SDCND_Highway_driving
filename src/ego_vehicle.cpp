#include "ego_vehicle.h"

ego_vehicle::ego_vehicle() {
    current_cycle = 1;
    current_speed_s = 0.0;
    current_speed_d = 0.0;
    current_acc_s = 0.0;
    current_acc_d = 0.0;
    current_lane = 1;
    ref_speed_xy = SPEED_LIMIT;
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

vector<vector<double>> ego_vehicle::SplineTraj(double x0, double y0, double th0, double v0, double s0,
        const vector<double> &previousXpoints, const vector<double> &previousYpoints, 
        const vector<double> &mapsS, const vector<double> &mapsX, const vector<double> &mapsY) {
    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later these points are used in order to create a smooth trajectory using spline interpolation
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
    vector<double> new_spline_point_1 = getXY(s0 + 30,(2+4*current_lane), mapsS, mapsX, mapsY);
    vector<double> new_spline_point_2 = getXY(s0 + 60,(2+4*current_lane), mapsS, mapsX, mapsY);
    vector<double> new_spline_point_3 = getXY(s0 + 90,(2+4*current_lane), mapsS, mapsX, mapsY);

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
    for (unsigned int i = 1; i <= 50 - previousXpoints.size(); i++) {
        double N = target_dist/(TIME_STEP*ref_speed_xy);
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