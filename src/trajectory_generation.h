/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include "helpers.h"
#include "simulation_data.h"

#include <vector>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const Ego& ego,
                        TrajectoryValues& traj,
                        const HighwayMap& map_waypoints,
                        const Predictions& predictions,
                        const State& state);

    /*
     * Given a possible next state, generate the appropriate trajectory to realize the next state.
     */
    void process();

private:
    /*
     * Generate a keep lane trajectory.
     */
    void keep_lane_trajectory();

    /*
     * Generate a lane change trajectory.
     */
    void lane_change_trajectory(const State& state);

    const Ego& ego_;
    TrajectoryValues& traj_;
    const HighwayMap& map_;
    const Predictions& predictions_;
    const State& future_state_;
};

#endif // TRAJECTORY_GENERATION_H
