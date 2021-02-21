/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef SIMULATION_DATA_H
#define SIMULATION_DATA_H

#include "parameters.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

// global variables
extern double g_ego_vel; // mps
extern int g_ego_lane;

// forward declaration for the typedefs
class Vehicle;

using State = std::string;
using States = std::vector<State>;
using Predictions = std::vector<Vehicle>; // map of the environment around ego at every 0.02 s

struct TrajectoryValues
{
    TrajectoryValues() = default;

    int ego_lane{g_ego_lane};
    double ego_vel{g_ego_vel};
    State ego_state{"KL"};
    double speed_diff{0.};
    bool new_value{false};
};

// based on the current state of the ego vehicle, helps to steer towards other lane
static const std::unordered_map<State, int> kLaneDirections{{{"LCL", -1}, {"LCR", +1}}};

/**
 * data from simulator simulating sensor_fusion module about traffic on road around ego
 */
struct Vehicle
{
    double id_{}; // unique identifier for the car
    int lane_{};
    double x_{}, y_{};  // location in global map coordinates, in m
    double s_{}, d_{};  // Frenet coordinates for that car, in m
    double yaw_{};      // yaw angle in the map, in radian
    double speed_{};    // speed in global map coordinates, in m/s
    State state_{"KL"}; // current state of ego

    Vehicle() = default;
    explicit Vehicle(const std::vector<double>& v);
    Vehicle(int lane, double s, double speed,
            std::string state = "KL"); // useful to store the trajectory for ego

    /*
     * Position of a vehicle after t time from now, assuming constant velocity motion model
     */
    double position_at(double t) const;

    /*
     * get the next position after dt time
     */
    void increment_position(double dt = kUnitTime);
};

/**
 * sensor_fusion data about self driving car and its surrounding traffic
 */
struct Ego : public Vehicle
{
    // The ego path that was sent to the simulator last time, but with processed points removed (<= 50 points)
    // can be helpful to see how far along the path has processed since last time.
    std::vector<double> prev_path_x_, prev_path_y_;

    // Ego's previous path's (given to the simulator) last point's Frenet s-d values
    double end_path_s_, end_path_d_;

    // Sensor Fusion Data, a list of all other car's attributes on the same side
    // of the road. (No Noise)
    std::vector<Vehicle> agent_cars_{};

    Ego(double x,
        double y,
        double s,
        double d,
        double yaw,   // given in degree
        double speed, // given in mph
        std::vector<double> prev_path_x,
        std::vector<double> prev_path_y,
        double end_path_s,
        double end_path_d,
        const std::vector<std::vector<double>>& sensor_fusion);
};

#endif
