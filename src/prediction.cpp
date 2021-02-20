/*
 * @Author: Udacity
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-18 19:59:49
 */

#include "prediction.h"
#include "simulation_data.h"

#include <memory>

Predictor::Predictor(Ego& ego, Predictions& predictions)
    : ego_{ego}
    , predictions_{predictions}
{
}

void Predictor::process() const
{
    // Note that we are not really "tracking" the vehicles with their ids, but
    // merely creating all the vehicles at every timestamp with their speed, lane
    // number, position on the road. This is like a "snapshot" of the environment
    // around ego at every timestamp (0.02s)

    // Generates predictions for non-ego vehicles to be used in trajectory generation for the ego vehicle.
    for (auto& agent_car : ego_.agent_cars_)
    {
        // agent's position after leftover points from previous path of ego
        // this is because we will make ego car follow the leftover trajectory from previous timestamp
        // to increase smoothness of its current trajectory
        agent_car.s_ = agent_car.position_at(kUnitTime * ego_.prev_path_x_.size()); // constant velocity model
    }

    // to reuse space, assign the vector of agents to predictions
    predictions_ = std::move(ego_.agent_cars_);
}
