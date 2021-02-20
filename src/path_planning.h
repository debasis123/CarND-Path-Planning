/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "helpers.h"
#include "simulation_data.h"

#include <iostream>
#include <vector>

/**
 * \brief Runs the path planner module
 */
class PathPlanner
{
public:
    PathPlanner(Ego& ego,
                std::vector<double>& next_x_vals,
                std::vector<double>& next_y_vals,
                const HighwayMap& map_waypoints);

    void run();

private:
    struct RefValues
    {
        double x{}, y{}, yaw{};
    };

    void add_past_anchor_waypoints();
    void add_future_anchor_waypoints();
    void transform_trajectory_to_car_coordinates();
    void compute_reference_values();
    /*
     * create a spline out of the trajectory points
     */
    void create_spline_trajectory();
    void transform_trajectory_to_global_coordinates();

    Ego& ego_;
    std::vector<double>& next_x_vals_;
    std::vector<double>& next_y_vals_;
    const HighwayMap& map_;
    std::vector<double> traj_x_{};
    std::vector<double> traj_y_{};
    TrajectoryValues traj_{};
    RefValues ref_{};
};

#endif // PATH_PLANNER_H
