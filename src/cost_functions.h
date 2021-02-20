/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include "simulation_data.h"

#include <map>
#include <string>

class CostCalculator
{
public:
    CostCalculator(const Ego& ego, const TrajectoryValues& traj, const Predictions& predictions);

    /*
     * Sum weighted cost functions to get total cost for trajectory.
     */
    double calculate_cost() const;

private:
    /*
     * Generate helper data to use in cost functions:
     * indended_lane: +/- 1 from the current lane if the vehicle is planning or executing a lane change.
     * final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR
     * trajectories.
     * distance_to_goal: The s distance of the vehicle to the goal.
     * Note that indended_lane and final_lane are both included to help differentiate
     * between planning and executing a lane change in the cost functions.
     */
    std::map<std::string, double> get_helper_data() const;

    const Ego& ego_;
    const TrajectoryValues& traj_;
    const Predictions& predictions_;
};

#endif // COST_FUNCTIONS_H
