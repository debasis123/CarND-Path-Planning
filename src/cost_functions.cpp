/*
 * @Author: debasis
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-21 08:58:22
 */

#include "cost_functions.h"
#include "prediction.h"
#include "simulation_data.h"

#include <algorithm>
#include <cmath>
#include <functional>
// #include <iostream>
#include <iterator>
#include <map>

const double REACH_GOAL = 0.5;
const double EFFICIENCY = 0.5;

CostCalculator::CostCalculator(const Ego& ego, const TrajectoryValues& traj, const Predictions& predictions)
    : ego_{ego}
    , traj_{traj}
    , predictions_{predictions}
{
}

/*
 * Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
 * Cost of being out of goal lane also becomes larger as vehicle approaches goal distance. This function is
 * very similar to what you have already implemented in the "Implement a Cost Function in C++" quiz.
 */
static double goal_distance_cost(const Predictions& predictions, const std::map<std::string, double>& data)
{
    double cost;
    double distance = data.at("distance_to_goal");
    if (distance > 0)
    {
        const double delta_d =
          std::abs(kEgoTargetLane - data.at("intended_lane")) + std::abs(kEgoTargetLane - data.at("final_lane"));
        cost = 1 - 2 * exp(-delta_d / distance);
    }
    else
    {
        cost = 1;
    }
    // std::cout << "distance cost: " << cost << '\n';

    return cost;
}

/*
 * All non ego vehicles in a lane have the same speed, so to get the speed limit
 * for a lane, we can just find one vehicle in that lane.
 */
static double lane_speed(const Predictions& predictions, int lane)
{
    auto itr = std::find_if(
      predictions.cbegin(), predictions.cend(), [&](const Vehicle& vehicle) { return vehicle.lane_ == lane; });

    // if found no vehicle in the lane, return -1
    return itr != predictions.cend() ? itr->speed_ : -1.0;
}

/*
 * Cost becomes higher for trajectories with intended lane and final lane that
 * have traffic slower than vehicle's target speed. You can use the
 * lane_speed(const std::map<int, std::vector<Vehicle>> & predictions, int lane) function
 * to determine the speed for a lane. This function is very similar to what you
 * have already implemented in the "Implement a Second Cost Function in C++"
 * quiz.
 */
static double inefficiency_cost(const Predictions& predictions, const std::map<std::string, double>& data)
{
    auto proposed_speed_intended = lane_speed(predictions, data.at("intended_lane"));

    // If no vehicle is in the proposed lane, we can travel at target speed.
    if (proposed_speed_intended < 0)
    {
        proposed_speed_intended = kTargetEgoSpeed;
    }

    double proposed_speed_final = lane_speed(predictions, data.at("final_lane"));
    if (proposed_speed_final < 0)
    {
        proposed_speed_final = kTargetEgoSpeed;
    }

    double cost =
      ((kTargetEgoSpeed - proposed_speed_intended) + (kTargetEgoSpeed - proposed_speed_final)) / kTargetEgoSpeed;

    // std::cout << "inefficiency cost: " << cost << '\n';

    return cost;
}

std::map<std::string, double> CostCalculator::get_helper_data() const
{
    // auto trajectory_last = traj_.at(0); // there is only next point of ego

    double intended_lane = traj_.ego_lane;
    double final_lane = traj_.ego_lane;
    double distance_to_goal = kHighwayDistance - ego_.s_; // using ego.s, that's not exactly correct though

    std::map<std::string, double> trajectory_data;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    return trajectory_data;
}

double CostCalculator::calculate_cost() const
{
    auto trajectory_data = get_helper_data();

    // Add additional cost functions here.
    std::vector<std::function<double(const Predictions&, const std::map<std::string, double>&)>> cf_list = {
      goal_distance_cost, inefficiency_cost};

    std::vector<double> weight_list = {REACH_GOAL, EFFICIENCY};

    double cost = 0.0;
    for (int i = 0; i < cf_list.size(); ++i)
    {
        double new_cost = weight_list.at(i) * cf_list.at(i)(predictions_, trajectory_data);
        cost += new_cost;
        // std::cout << "new_cost: " << new_cost << "      cost: " << cost << '\n';
    }

    return cost;
}
