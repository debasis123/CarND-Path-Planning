/*
 * @Author: Udacity
 * @Last Modified by:   Debasis Mandal
 */

#include "helpers.h"
#include "path_planning.h"
#include "simulation_data.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <uWS/uWS.h>

// #include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
// #include <thread>
#include <vector>

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
static std::string hasData(const std::string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != std::string::npos)
    {
        return "";
    }
    if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    // ====================== read map data from file =============

    // Waypoints data on the highway; read from a file
    // provided in GLOBAL x,y as well as Frenet's s,d coordinates
    std::string map_file("../data/highway_map.csv");

    // Load up map values for waypoint's x, y, s and d normalized normal vectors from the "highway_map.csv"
    // These values are for the left edge of the left most lane on the road along the direction of the ego car
    std::ifstream ifs(map_file.c_str(), std::ifstream::in);
    HighwayMap map_waypoints(ifs);

    h.onMessage([&map_waypoints](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // ====================== read data from simulator =============

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        // auto sdata = std::string(data).substr(0, length);
        // std::cout << sdata << std::endl;
        if (length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);
            if (!s.empty())
            {
                auto j = json::parse(s);
                auto event = j[0].get<std::string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Ego car's localization and environment Data in the simulation
                    Ego ego(j[1]["x"],
                            j[1]["y"],
                            j[1]["s"],
                            j[1]["d"],
                            j[1]["yaw"],
                            j[1]["speed"],
                            j[1]["previous_path_x"],
                            j[1]["previous_path_y"],
                            j[1]["end_path_s"],
                            j[1]["end_path_d"],
                            j[1]["sensor_fusion"]);

                    // ====================== run path planner ===========================

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02
                    // seconds (20 ms).

                    // Note that the ego car uses a perfect controller and will visit every (x,y) point it receives in
                    // the list every .02 seconds. The units for the (x,y) points are in meters.
                    // Also note that the spacing of the points determines the *speed* of the ego car. The vector
                    // going from a point to the next point in the list dictates the *yaw* of the car. So all the path
                    // planner needs to do is filling up the following two vectors.

                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;
                    PathPlanner planner(ego, next_x_vals, next_y_vals, map_waypoints);
                    planner.run();

                    // =================== send data back to simulator ==================

                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    // this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP
    // but if it's removed the program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection(
      [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) { ws.close(); });

    const int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
