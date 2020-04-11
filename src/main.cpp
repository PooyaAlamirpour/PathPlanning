
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "smoother.h"
#include "constants.h"
#include "vehicle.h"
#include "costs.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    double heading = atan2( (map_y-y),(map_x-x) );
    double angle = abs(theta-heading);
    if(angle > pi()/4)
    {
        closestWaypoint++;
    }
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    // calculate s value
    double frenet_s = maps_s[0];
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    frenet_s += distance(0,0,proj_x,proj_y);
    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
    int prev_wp = -1;
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    int wp2 = (prev_wp+1)%maps_x.size();
    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);
    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    double perp_heading = heading-pi()/2;
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    return {x,y};
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    Vehicle my_car = Vehicle();

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);
    ofstream log_file;
    log_file.open("path_planning_log.csv");

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &my_car, &log_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                 uWS::OpCode opCode) {
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
                    car_speed *= 0.44704;  														// convert mph to m/s
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    ofstream single_iteration_log;
                    single_iteration_log.open("path_planning_log-single_iteration.csv");

                    int num_waypoints = map_waypoints_x.size();
                    int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
                    vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
                            coarse_waypoints_dx, coarse_waypoints_dy;
                    for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++) {
                        // for smooting, take so many previous and so many subsequent waypoints
                        int idx = (next_waypoint_index+i) % num_waypoints;
                        if (idx < 0) {
                            // correct for wrap
                            idx += num_waypoints;
                        }
                        // correct for wrap in s for spline interpolation (must be continuous)
                        double current_s = map_waypoints_s[idx];
                        double base_s = map_waypoints_s[next_waypoint_index];
                        if (i < 0 && current_s > base_s) {
                            current_s -= TRACK_LENGTH;
                        }
                        if (i > 0 && current_s < base_s) {
                            current_s += TRACK_LENGTH;
                        }
                        coarse_waypoints_s.push_back(current_s);
                        coarse_waypoints_x.push_back(map_waypoints_x[idx]);
                        coarse_waypoints_y.push_back(map_waypoints_y[idx]);
                        coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
                        coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
                    }

                    // interpolation parameters
                    double dist_inc = 0.5;
                    int num_interpolation_points = (coarse_waypoints_s[coarse_waypoints_s.size()-1] - coarse_waypoints_s[0]) / dist_inc;
                    vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y,
                            interpolated_waypoints_dx, interpolated_waypoints_dy;
                    // interpolated s is simply...
                    interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
                    for (int i = 1; i < num_interpolation_points; i++) {
                        interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
                    }
                    interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc, num_interpolation_points);
                    interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc, num_interpolation_points);
                    interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc, num_interpolation_points);
                    interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc, num_interpolation_points);

                    double pos_s, s_dot, s_ddot;
                    double pos_d, d_dot, d_ddot;

                    // Other values necessary for determining these based on future points in previous path
                    double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
                            pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;

                    int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
                    double traj_start_time = subpath_size * PATH_DT;

                    // use default values if not enough previous path points
                    if (subpath_size < 4) {
                        pos_x = car_x;
                        pos_y = car_y;
                        angle = deg2rad(car_yaw);
                        pos_s = car_s;
                        pos_d = car_d;
                        s_dot = car_speed;
                        d_dot = 0;
                        s_ddot = 0;
                        d_ddot = 0;
                    } else {
                        // consider current position to be last point of previous path to be kept
                        pos_x = previous_path_x[subpath_size-1];
                        pos_y = previous_path_y[subpath_size-1];
                        pos_x2 = previous_path_x[subpath_size-2];
                        pos_y2 = previous_path_y[subpath_size-2];
                        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
                        vector<double> frenet = getFrenet(pos_x, pos_y, angle, interpolated_waypoints_x, interpolated_waypoints_y, interpolated_waypoints_s);
                        //vector<double> frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y, map_waypoints_s);
                        pos_s = frenet[0];
                        pos_d = frenet[1];

                        int next_interp_waypoint_index = NextWaypoint(pos_x, pos_y, angle, interpolated_waypoints_x,
                                                                      interpolated_waypoints_y);
                        double dx = interpolated_waypoints_dx[next_interp_waypoint_index - 1];
                        double dy = interpolated_waypoints_dy[next_interp_waypoint_index - 1];
                        // sx,sy vector is perpendicular to dx,dy
                        double sx = -dy;
                        double sy = dx;

                        // calculate s_dot & d_dot
                        vel_x1 = (pos_x - pos_x2) / PATH_DT;
                        vel_y1 = (pos_y - pos_y2) / PATH_DT;
                        // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
                        // and D are unit vectors this is simply the dot products of V with S and V with D
                        s_dot = vel_x1 * sx + vel_y1 * sy;
                        d_dot = vel_x1 * dx + vel_y1 * dy;

                        // have to get another point to calculate s_ddot, d_ddot from xy acceleration
                        pos_x3 = previous_path_x[subpath_size-3];
                        pos_y3 = previous_path_y[subpath_size-3];
                        vel_x2 = (pos_x2 - pos_x3) / PATH_DT;
                        vel_y2 = (pos_y2 - pos_y3) / PATH_DT;
                        acc_x = (vel_x1 - vel_x2) / PATH_DT;
                        acc_y = (vel_y1 - vel_y2) / PATH_DT;
                        s_ddot = acc_x * sx + acc_y * sy;
                        d_ddot = acc_x * dx + acc_y * dy;

                        double eval_time, pos_s2, pos_d2, s_dot2, d_dot2, s_ddot2, d_ddot2;
                        vector<double> s_dot_coeffs = my_car.differentiate_coeffs(my_car.s_traj_coeffs);
                        vector<double> d_dot_coeffs = my_car.differentiate_coeffs(my_car.d_traj_coeffs);
                        vector<double> s_ddot_coeffs = my_car.differentiate_coeffs(s_dot_coeffs);
                        vector<double> d_ddot_coeffs = my_car.differentiate_coeffs(d_dot_coeffs);
                        eval_time = (NUM_PATH_POINTS - subpath_size) * PATH_DT;
                        pos_s2 = my_car.evaluate_coeffs_at_time(my_car.s_traj_coeffs, eval_time);
                        pos_d2 = my_car.evaluate_coeffs_at_time(my_car.d_traj_coeffs, eval_time);
                        s_dot2 = my_car.evaluate_coeffs_at_time(s_dot_coeffs, eval_time);
                        d_dot2 = my_car.evaluate_coeffs_at_time(d_dot_coeffs, eval_time);
                        s_ddot2 = my_car.evaluate_coeffs_at_time(s_ddot_coeffs, eval_time);
                        d_ddot2 = my_car.evaluate_coeffs_at_time(d_ddot_coeffs, eval_time);
                    }

                    my_car.s    = pos_s;           // s position
                    my_car.s_d  = s_dot;           // s dot - velocity in s
                    my_car.s_dd = s_ddot;          // s dot-dot - acceleration in s
                    my_car.d    = pos_d;           // d position
                    my_car.d_d  = d_dot;           // d dot - velocity in d
                    my_car.d_dd = d_ddot;          // d dot-dot - acceleration in d

                    double duration = N_SAMPLES * DT - subpath_size * PATH_DT;
                    vector<Vehicle> other_cars;
                    map<int, vector<vector<double>>> predictions;
                    for (auto sf: sensor_fusion) {
                        double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
                        Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
                        other_cars.push_back(other_car);
                        int v_id = sf[0];
                        vector<vector<double>> preds = other_car.generate_predictions(traj_start_time, duration);
                        predictions[v_id] = preds;
                    }

                    bool car_to_left = false, car_to_right = false, car_just_ahead = false;
                    for (Vehicle other_car: other_cars) {
                        double s_diff = fabs(other_car.s - car_s);
                        if (s_diff < FOLLOW_DISTANCE) {
                            cout << "s diff: " << s_diff << endl;
                            double d_diff = other_car.d - car_d;
                            if (d_diff > 2 && d_diff < 6) {
                                car_to_right = true;
                            } else if (d_diff < -2 && d_diff > -6) {
                                car_to_left = true;
                            } else if (d_diff > -2 && d_diff < 2) {
                                car_just_ahead = true;
                            }
                        }
                    }

                    my_car.update_available_states(car_to_left, car_to_right);

                    vector<vector<double>> best_frenet_traj, best_target;
                    double best_cost = 999999;
                    string best_traj_state = "";
                    for (string state: my_car.available_states) {
                        vector<vector<double>> target_s_and_d = my_car.get_target_for_state(state, predictions, duration, car_just_ahead);
                        vector<vector<double>> possible_traj = my_car.generate_traj_for_target(target_s_and_d, duration);
                        double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions);

                        if (current_cost < best_cost) {
                            best_cost = current_cost;
                            best_frenet_traj = possible_traj;
                            best_traj_state = state;
                            best_target = target_s_and_d;
                        }
                    }

                    single_iteration_log << "i,ego s,ego d,s1,d1,s2,d2,s3,d3,s4,d4,s5,d5,s6,d6,s7,d7,s8,d8,s9,d9,s10,d10,s11,d11,s12,d12" << endl;
                    for (int i = 0; i < best_frenet_traj[0].size(); i++) {
                        single_iteration_log << i << ",";
                        single_iteration_log << best_frenet_traj[0][i] << "," << best_frenet_traj[1][i] << ",";
                        for (auto prediction : predictions) {
                            vector<vector<double>> pred_traj = prediction.second;
                            single_iteration_log << pred_traj[i][0] << "," << pred_traj[i][1] << ",";
                        }
                        single_iteration_log << endl;
                    }

                    vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj, interpolated_s_traj,
                            interpolated_x_traj, interpolated_y_traj;

                    double prev_s = pos_s - s_dot * PATH_DT;

                    // first two points of coarse trajectory, to ensure spline begins smoothly
                    if (subpath_size >= 2) {
                        coarse_s_traj.push_back(prev_s);
                        coarse_x_traj.push_back(previous_path_x[subpath_size-2]);
                        coarse_y_traj.push_back(previous_path_y[subpath_size-2]);
                        coarse_s_traj.push_back(pos_s);
                        coarse_x_traj.push_back(previous_path_x[subpath_size-1]);
                        coarse_y_traj.push_back(previous_path_y[subpath_size-1]);
                    } else {
                        double prev_s = pos_s - 1;
                        double prev_x = pos_x - cos(angle);
                        double prev_y = pos_y - sin(angle);
                        coarse_s_traj.push_back(prev_s);
                        coarse_x_traj.push_back(prev_x);
                        coarse_y_traj.push_back(prev_y);
                        coarse_s_traj.push_back(pos_s);
                        coarse_x_traj.push_back(pos_x);
                        coarse_y_traj.push_back(pos_y);
                    }

                    // last two points of coarse trajectory, use target_d and current s + 30,60
                    double target_s1 = pos_s + 30;
                    double target_d1 = best_target[1][0];
                    vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                    double target_x1 = target_xy1[0];
                    double target_y1 = target_xy1[1];
                    coarse_s_traj.push_back(target_s1);
                    coarse_x_traj.push_back(target_x1);
                    coarse_y_traj.push_back(target_y1);
                    double target_s2 = target_s1 + 30;
                    double target_d2 = target_d1;
                    vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
                    double target_x2 = target_xy2[0];
                    double target_y2 = target_xy2[1];
                    coarse_s_traj.push_back(target_s2);
                    coarse_x_traj.push_back(target_x2);
                    coarse_y_traj.push_back(target_y2);

                    // next s values
                    double target_s_dot = best_target[0][1];
                    double current_s = pos_s;
                    double current_v = s_dot;
                    double current_a = s_ddot;
                    for (int i = 0; i < (NUM_PATH_POINTS - subpath_size); i++) {
                        double v_incr, a_incr;
                        if (fabs(target_s_dot - current_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
                            v_incr = 0;
                        } else {
                            v_incr = (target_s_dot - current_v)/(fabs(target_s_dot - current_v)) * VELOCITY_INCREMENT_LIMIT;
                        }
                        current_v += v_incr;
                        current_s += current_v * PATH_DT;
                        interpolated_s_traj.push_back(current_s);
                    }

                    interpolated_x_traj = interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
                    interpolated_y_traj = interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);

                    // add previous path, if any, to next path
                    for(int i = 0; i < subpath_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    // add xy points from newly generated path
                    for (int i = 0; i < interpolated_x_traj.size(); i++) {
                        //if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
                        next_x_vals.push_back(interpolated_x_traj[i]);
                        next_y_vals.push_back(interpolated_y_traj[i]);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    single_iteration_log.close();
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;

        // DEBUG
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 80; j++) {
                cout << "#";
            }
            cout << endl;
        }
        cout << endl;
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

    log_file.close();
}