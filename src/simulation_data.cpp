/*
 * @Author: Debasis Mandal
 * @Date:   2021-02-10 17:51:23
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-20 16:59:29
 */

#include "simulation_data.h"
#include "helpers.h"

#include <algorithm>
// #include <iostream>

double g_ego_vel{0}; // mps
int g_ego_lane{kEgoStartLane};

Vehicle::Vehicle(int lane, double s, double speed, std::string state)
    : lane_{lane}
    , s_{s}
    , speed_{speed}
    , state_{std::move(state)}
{
}

Vehicle::Vehicle(const std::vector<double>& v)
    : id_(v.at(0))
    , x_(v.at(1))
    , y_(v.at(2))
    , s_(v.at(5))
    , d_(v.at(6))
    , speed_(std::sqrt(v.at(3) * v.at(3) + v.at(4) * v.at(4))) // values given in m/s
    , yaw_(std::atan2(v.at(4), v.at(3)))
{
    lane_ = static_cast<int>(d_ / kLaneWidth);
}

double Vehicle::position_at(double t) const
{
    return s_ + speed_ * t; // s = s_0 + u*t
}

void Vehicle::increment_position(double dt)
{
    s_ = position_at(dt);
}

Ego::Ego(double x,
         double y,
         double s,
         double d,
         double yaw,   // given in degree
         double speed, // given in mph
         std::vector<double> prev_path_x,
         std::vector<double> prev_path_y,
         double end_path_s,
         double end_path_d,
         const std::vector<std::vector<double>>& sensor_fusion)
    : Vehicle(kEgoStartLane, s, kMphToMps * speed)
    , prev_path_x_{std::move(prev_path_x)}
    , prev_path_y_{std::move(prev_path_y)}
    , end_path_s_{end_path_s}
    , end_path_d_{end_path_d}
{
    id_ = -1.0; // ego's id
    x_ = x;
    y_ = y;
    d_ = d;
    yaw_ = deg2rad(yaw);

    // update lane and velocity to global variables
    // we will be using these values instead of the ones coming from simulator
    lane_ = g_ego_lane;
    speed_ = g_ego_vel;

    // set the data for other moving objects around ego
    std::for_each(sensor_fusion.cbegin(), sensor_fusion.cend(), [&](const std::vector<double>& carInfo) {
        agent_cars_.emplace_back(carInfo);
    });

    // std::cout << "\n\nINITIAL VALUES: " << lane_ << "         " << speed_ << '\n';
    // std::cout << "prev_path_size: " << prev_path_x_.size() << '\n';
}
