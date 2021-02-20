/*
 * @Author: Debasis Mandal
 * @Date:   2021-01-31 18:53:12
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-20 17:15:52
 */

#include "behavior_planning.h"
#include "cost_functions.h"
#include "helpers.h"
#include "simulation_data.h"
#include "trajectory_generation.h"

#include <algorithm>
#include <cassert>
// #include <iostream>
#include <vector>

BehaviorPlanner::BehaviorPlanner(const Ego& ego,
                                 TrajectoryValues& traj,
                                 const HighwayMap& map_waypoints,
                                 const Predictions& predictions)
    : ego_{ego}
    , traj_{traj}
    , map_waypoints_{map_waypoints}
    , predictions_{predictions}
{
}

void BehaviorPlanner::process()
{
    // accumulate all future possible states of ego car
    auto states = successor_states();

    std::vector<double> costs_of_states;
    std::vector<TrajectoryValues> final_trajectories;

    for (const auto& state : states)
    {
        TrajectoryValues traj{}; // initialized with proper values
        TrajectoryGenerator trajGen(ego_, traj, map_waypoints_, predictions_, state);
        trajGen.process();
        // std::cout << "traj: " << traj.ego_lane << "   " << traj.ego_vel << "   " << traj.ego_state << "    "
        // << traj.speed_diff << '\n';

        if (traj.new_value)
        {
            CostCalculator calc(ego_, traj, predictions_);
            auto cost_state = calc.calculate_cost();
            // std::cout << "state: " << state << "        cost: " << cost_state << '\n';
            costs_of_states.push_back(cost_state);
            final_trajectories.push_back(std::move(traj));
        }
    }
    // print_vector(costs_of_states);

    // find the trajectory with the lowest cost
    auto best_cost = std::min_element(std::begin(costs_of_states), std::end(costs_of_states));
    auto best_idx = std::distance(std::begin(costs_of_states), best_cost);
    // std::cout << *best_cost << "        " << best_idx << '\n';

    // add the lowest cost trajectories
    traj_ = std::move(final_trajectories.at(best_idx));

    // std::cout << "best_trajectory: " << traj_.ego_lane << "   " << traj_.ego_vel << "   " << traj_.ego_state << " "
    // << traj_.speed_diff << '\n';
}

States BehaviorPlanner::successor_states() const
{
    States states;

    // by default, maintain lane
    states.push_back("KL");

    State state = ego_.state_;

    if (state == "KL")
    {
        if (is_vehicle_ahead(predictions_, ego_))
        {
            if (!is_vehicle_left(predictions_, ego_))
            {
                states.push_back("LCL");
            }
            else if (!is_vehicle_right(predictions_, ego_))
            {
                states.push_back("LCR");
            }
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}
