/*
 * @Author: debasis
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-20 16:18:59
 */

#include "helpers.h"
#include "simulation_data.h"
#include "trajectory_generation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

TrajectoryGenerator::TrajectoryGenerator(const Ego& ego,
                                         TrajectoryValues& traj,
                                         const HighwayMap& map_waypoints,
                                         const Predictions& predictions,
                                         const State& state)
    : ego_{ego}
    , traj_{traj}
    , map_{map_waypoints}
    , predictions_{predictions}
    , future_state_{state}
{
}

void TrajectoryGenerator::process()
{
    // The ego car uses a perfect controller and will visit every (x,y) point in the trajectory
    // it receives in the list every .02 seconds.

    // generate one trajectory based on the future state of ego car
    // first, we generate trajectories only in s, d values, Note that these are GLOBAL COORDINATES

    // GOAL: To go on the FASTEST LANE close to the speed limit of 50 mph
    if (future_state_ == "KL")
    {
        keep_lane_trajectory();
    }
    else if (future_state_ == "LCL" || future_state_ == "LCR")
    {
        lane_change_trajectory(future_state_);
    }
}

void TrajectoryGenerator::keep_lane_trajectory()
{
    // position at next timestamp
    traj_ = {ego_.lane_, ego_.speed_, future_state_, 0, true};

    if (is_vehicle_ahead(predictions_, ego_))
    {
        if (ego_.lane_ > 0 && !is_vehicle_left(predictions_, ego_))
        {
            lane_change_trajectory("LCL");
        }
        else if (ego_.lane_ < kNumLanes - 1 && !is_vehicle_right(predictions_, ego_))
        {
            lane_change_trajectory("LCR");
        }
        else
        {
            // traj_.speed_diff -= kMaxAcceleration * kUnitTime;
            traj_.speed_diff -= kMaxDeceleration * kUnitTime;
        }
    }
    else
    {
        // increase speed
        if (ego_.speed_ < kTargetEgoSpeed)
        {
            traj_.speed_diff += kMaxAcceleration * kUnitTime;
        }
    }
}

void TrajectoryGenerator::lane_change_trajectory(const State& state)
{
    // position at next timestamp
    auto new_lane = ego_.lane_ + kLaneDirections.at(state);

    // if goes outside road, maintain current lane
    if (new_lane < 0 || new_lane > kNumLanes - 1)
    {
        // maintain same lane
        // is taken care in the behavior planning by default
        return;
    }

    traj_ = {new_lane, ego_.speed_, state, 0, true};
}
