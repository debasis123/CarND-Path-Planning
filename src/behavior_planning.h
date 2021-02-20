/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef BEHAVIOR_PLANNING_H
#define BEHAVIOR_PLANNING_H

#include "helpers.h"
#include "simulation_data.h"
#include <vector>

/**
 * \brief Behavior planner for the ego vehicle
 */
class BehaviorPlanner
{
public:
    BehaviorPlanner(const Ego& ego,
                    TrajectoryValues& traj,
                    const HighwayMap& map_waypoints,
                    const Predictions& predictions);
    /**
     * transition function for the FSM from the Behavior Planning
     *
     * @param A predictions map. This is a map of vehicle id keys with predicted
     *   vehicle trajectories as values. Trajectories are a vector of Vehicle
     *   objects representing the vehicle at the current timestep and one timestep
     *   in the future.
     * @output The best (lowest cost) trajectory corresponding to the next ego
     *   vehicle state.
     */
    void process();

private:
    /*
     * Provides the possible next states given the current state of ego
     * FSM from the course
     */
    States successor_states() const;

    const Ego& ego_;
    TrajectoryValues& traj_;
    const HighwayMap& map_waypoints_;
    const Predictions& predictions_;
};

#endif // BEHAVIOR_PLANNING_H
