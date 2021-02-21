/*
 * @Author: debasis123
 * @Last Modified by:   debasis123
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "simulation_data.h"

#include <cmath>
#include <fstream>
// #include <iostream>
#include <sstream>
#include <string>
#include <vector>

using DoublePair = std::pair<double, double>;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

void print_vector(const std::vector<double>& v);

// Closest point can be in the front or in the back
int closestWaypoint(double x, double y, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

// the next waypoint is in the front only
int nextWaypoint(double x,
                 double y,
                 double theta,
                 const std::vector<double>& maps_x,
                 const std::vector<double>& maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
DoublePair getFrenetFromXY(double x,
                           double y,
                           double theta,
                           const std::vector<double>& maps_x,
                           const std::vector<double>& maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
DoublePair getXYFromFrenet(double s,
                           double d,
                           const std::vector<double>& maps_s,
                           const std::vector<double>& maps_x,
                           const std::vector<double>& maps_y);

struct HighwayMap
{
    // in global map X-Y coordinates
    std::vector<double> x_, y_;
    // in Frenet coordinates; d split up into the global x and y components
    std::vector<double> s_, dx_, dy_;

    explicit HighwayMap(std::ifstream& ifs);
};

bool is_vehicle_left(const Predictions& predictions, const Ego& ego);
bool is_vehicle_right(const Predictions& predictions, const Ego& ego);
bool is_vehicle_ahead(const Predictions& predictions, const Ego& ego);

#endif // HELPERS_H
