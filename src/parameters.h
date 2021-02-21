/*
 * @Author: debasis123
 * @Last Modified by:   debasis123
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

// fixed parameters
static constexpr double kMphToMps{0.44704};
static constexpr double kUnitTime{0.02};              // in sec (= 20 ms)
static constexpr double kTargetEgoSpeed{22};          // in m/s ( < 50 mph)
static constexpr double kHighwayLength{6946.0};       // in m, maximum possible value of s for ego
static constexpr double kHighwayDistance{3 * 6946.0}; // in m, # of turns
static constexpr int kNumLanes{3};
static constexpr int kEgoTargetLane{1}; // middle lane
static constexpr int kEgoStartLane{1};
static constexpr int kLaneWidth{4}; // in m

// parameter to be tuned
static constexpr double kMaxAcceleration{8.0}; // in m/s^2
static constexpr double kMaxDeceleration{6.0}; // in m/s^2
// static constexpr double kMaxJerk{10.0};           // in m/s^3
static constexpr double kCollisionDistance{30.0}; // in m
static constexpr double kSafeBufferDist{25.0};    // in m
static constexpr int kNumFuturePoints{4};         // required in path planning

#endif // PARAMETERS_H
