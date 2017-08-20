#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 *
 * @param s
 * @return
 */
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

/**
 * Evaluate a polynomial.
 *
 * @param coeffs
 * @param x
 * @return
 */
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

/**
 * Fit a polynomial.
 * Adapted from
 * https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 *
 * @param xvals
 * @param yvals
 * @param order
 * @return
 */
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

/**
 * Transform from simulator coordinates to vehicle coordinate system
 *
 * @param px            - x of vehicle in map's coordinates
 * @param py            - y of vehicle in map's coordinates
 * @param psi           - pis of vehicle in map's coordinates
 * @param ptsx          - x of points to transform
 * @param ptsy          - y of points to transform
 * @param ptsx_local    - transformed x
 * @param ptsy_local    - transformed y
 */
void transformToVehicleCoordinates(double px, double py, double psi,
                      const vector<double> &ptsx, const vector<double> &ptsy,
                      Eigen::VectorXd &ptsx_local, Eigen::VectorXd &ptsy_local)
{
    ptsx_local.resize(ptsx.size());
    ptsy_local.resize(ptsy.size());

    double cos_psi = cos(psi);
    double sin_psi = sin(psi);

    for (size_t i = 0; i < ptsx.size(); i++) {
        double diff_x = ptsx[i] - px;
        double diff_y = ptsy[i] - py;

        ptsx_local[i] = diff_x * cos_psi + diff_y * sin_psi;
        ptsy_local[i] = diff_y * cos_psi - diff_x * sin_psi;
    }
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    v *= 1609.344 / 3600; // convert to m/s

                    //
                    // Calculate steering angle and throttle using MPC.
                    // Both are in between [-1, 1].
                    //

                    //
                    // 1. Adjust vehicle coordinates, orientation and speed for latency
                    //
                    double latency = 0.1;
                    double x_latency = px + v * cos(psi) * latency;
                    double y_latency = py + v * sin(psi) * latency;
                    double psi_latency = psi - v / Lf * mpc.result.delta * latency;
                    double v_latency = v + mpc.result.a * latency;

                    //
                    // 2. Transform trajectory coordinates to vehicle coordinate system
                    //
                    Eigen::VectorXd ptsx_vehicle;
                    Eigen::VectorXd ptsy_vehicle;
                    transformToVehicleCoordinates(x_latency, y_latency, psi_latency, ptsx, ptsy,
                                                  ptsx_vehicle, ptsy_vehicle);

                    //
                    // 3. Fit polynomial
                    //
                    Eigen::VectorXd coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

                    //
                    // 4. Get cross-track and orientation errors
                    // assume that x_ref = y_ref = psi_ref = 0
                    //
                    double cte = polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]);

                    //
                    // 5. Set the state
                    //
                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v_latency, cte, epsi;

                    //
                    // 6. Optimize for steering and throttle
                    //
                    MPC::SolutionInfo &solution = mpc.Solve(state, coeffs);

                    // divide by deg2rad(25) to normalize for [-1, 1]
                    double steer_value = solution.delta / deg2rad(25);
                    double throttle_value = solution.a;

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = solution.x;
                    msgJson["mpc_y"] = solution.y;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //
                    // Adding (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    //
                    for ( int i = 0; i < ptsx_vehicle.size(); i++ ) {
                        next_x_vals.push_back(ptsx_vehicle[i]);
                        next_y_vals.push_back(ptsy_vehicle[i]);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    // SET TO 100 MILLISECONDS
                    //
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
