/*
 * @Author: Debasis Mandal
 * @Date:   2021-02-07 18:26:33
 * @Last Modified by:   Debasis Mandal
 * @Last Modified time: 2021-02-20 14:03:11
 */

#include "path_planning.h"

#include "helpers.h"
#include "simulation_data.h"
#include "prediction.h"
#include "behavior_planning.h"
#include "trajectory_generation.h"

#include "spline.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>

PathPlanner::PathPlanner(Ego& ego,
                         std::vector<double>& next_x_vals,
                         std::vector<double>& next_y_vals,
                         const HighwayMap& map_waypoints)
    : ego_{ego}
    , next_x_vals_{next_x_vals}
    , next_y_vals_{next_y_vals}
    , map_{map_waypoints}
{
}

void PathPlanner::run()
{
    // TODO: algorithm

    // Place the ego car at the end of the previous trajectory, if exists, since we will first follow the previous
    // trajectory by default and then add few points after that.
    if (!ego_.prev_path_x_.empty())
    {
        ego_.s_ = ego_.end_path_s_;
        // std::cout << ego_.s_ << "               " << ego_.end_path_s_ << '\n';
    }

    // =================== 1. Prediction ===================

    // generate predictions for each vehicle on the road for next 0.02 s = 20 ms
    Predictions predictions;
    Predictor predictor(ego_, predictions);
    predictor.process();

    // =================== 2. Behavior Planning and 3. Trajectory Generation ===================
    BehaviorPlanner behavPlanner(ego_, traj_, map_, predictions);
    behavPlanner.process();

    // std::cout << traj_.ego_lane << "   " << traj_.ego_vel << "   " << traj_.ego_state << "    " << traj_.speed_diff
    // << '\n';

    // first create some anchor points to create the spline trajectory
    add_past_anchor_waypoints();
    add_future_anchor_waypoints();
    compute_reference_values();
    // and now we transform the s,d values to car coordinates (x, y) for creating spline
    transform_trajectory_to_car_coordinates();
    create_spline_trajectory();
    // back to global coordinate
    transform_trajectory_to_global_coordinates();

    // update static variables for next round of simulation
    g_ego_vel = traj_.ego_vel;
    g_ego_lane = traj_.ego_lane;

    // std::cout << "static values: " << g_ego_lane << "      " << g_ego_vel << '\n';
}

void PathPlanner::add_past_anchor_waypoints()
{
    // Most of the points from previous path was processed
    // so do some interpolation from current position to create smoother trajectory
    auto prev_path_size = ego_.prev_path_x_.size();
    if (prev_path_size < 2)
    {
        // add the current position and interpolated previous one
        traj_x_.push_back(ego_.x_ - std::cos(ego_.yaw_));
        traj_y_.push_back(ego_.y_ - std::sin(ego_.yaw_));
        traj_x_.push_back(ego_.x_);
        traj_y_.push_back(ego_.y_);
    }
    else
    {
        // Use the last two points from the previous trajectory
        traj_x_.push_back(ego_.prev_path_x_.at(prev_path_size - 2));
        traj_y_.push_back(ego_.prev_path_y_.at(prev_path_size - 2));
        traj_x_.push_back(ego_.prev_path_x_.at(prev_path_size - 1));
        traj_y_.push_back(ego_.prev_path_y_.at(prev_path_size - 1));
    }

    // std::cout << "first traj_x and traj_y vals: \n";
    // print_vector(traj_x_);
    // print_vector(traj_y_);
    // auto xy = getXYFromFrenet(ego_.s_, ego_.d_, map_.s_, map_.x_, map_.y_);
    // std::cout << xy.first << "     " << xy.second << '\n';
}

// void PathPlanner::add_current_waypoints() {}

void PathPlanner::add_future_anchor_waypoints()
{
    // generate anchor points at distance of 30m, 60m, and 90m from ego position at the last waypoint from previous
    // trajectory
    auto ego_next_d = 2 + kLaneWidth * traj_.ego_lane;

    // std::cout << "lane: " << traj_.ego_lane << "   d: " << ego_next_d << "      s: " << ego_.s_ << '\n';

    for (int i{1}; i < 5; ++i)
    {
        auto xy = getXYFromFrenet(ego_.s_ + kCollisionDistance * i, ego_next_d, map_.s_, map_.x_, map_.y_);
        traj_x_.push_back(xy.first);
        traj_y_.push_back(xy.second);
    }

    // std::cout << "traj_x and traj_y vals before transformation: \n";
    // print_vector(traj_x_);
    // print_vector(traj_y_);
}

void PathPlanner::compute_reference_values()
{
    auto prev_path_size = ego_.prev_path_x_.size();
    if (prev_path_size < 2)
    {
        ref_.x = ego_.x_;
        ref_.y = ego_.y_;
        ref_.yaw = ego_.yaw_;
    }
    else
    {
        ref_.x = ego_.prev_path_x_.at(prev_path_size - 1);
        ref_.y = ego_.prev_path_y_.at(prev_path_size - 1);
        ref_.yaw = std::atan2(ref_.y - ego_.prev_path_y_.at(prev_path_size - 2),
                              ref_.x - ego_.prev_path_x_.at(prev_path_size - 2));
    }
}

void PathPlanner::transform_trajectory_to_car_coordinates()
{
    // Making coordinates to local car coordinates.
    for (size_t i{0}; i < traj_x_.size(); ++i)
    {
        // translation
        auto shift_x = traj_x_.at(i) - ref_.x;
        auto shift_y = traj_y_.at(i) - ref_.y;

        // rotation
        traj_x_[i] = shift_x * std::cos(0 - ref_.yaw) - shift_y * std::sin(0 - ref_.yaw);
        traj_y_[i] = shift_x * std::sin(0 - ref_.yaw) + shift_y * std::cos(0 - ref_.yaw);
    }

    // std::cout << "traj_x and traj_y vals: \n";
    // print_vector(traj_x_);
    // print_vector(traj_y_);
}

void PathPlanner::create_spline_trajectory()
{
    // Create the spline.
    tk::spline spl;
    spl.set_points(traj_x_, traj_y_);

    // std::cout << "anchor points set...\n";

    // For the smooth transition, we are adding previous path points
    auto prev_path_size = ego_.prev_path_x_.size();

    for (int i = 0; i < prev_path_size; ++i)
    {
        next_x_vals_.push_back(ego_.prev_path_x_.at(i));
        next_y_vals_.push_back(ego_.prev_path_y_.at(i));
    }

    // Calculate distance y position on 30 m ahead.
    double target_x = kCollisionDistance;
    double target_y = spl(target_x);
    double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

    double x_current = 0;

    for (int i = 0; i < 50 - prev_path_size; ++i)
    {
        traj_.ego_vel += traj_.speed_diff;
        traj_.ego_vel =
          (traj_.ego_vel > kTargetEgoSpeed)
            ? kTargetEgoSpeed
            : (traj_.ego_vel < kMaxAcceleration * kUnitTime) ? kMaxAcceleration * kUnitTime : traj_.ego_vel;

        // std::cout << traj_.ego_vel << '\n';

        double num_points = target_dist / (kUnitTime * traj_.ego_vel);
        double x_point = x_current + (target_x / num_points);
        double y_point = spl(x_point);

        x_current = x_point;

        next_x_vals_.push_back(x_point);
        next_y_vals_.push_back(y_point);
    }

    // std::cout << "before transformation: next_x and next_y vals: \n";
    // print_vector(next_x_vals_);
    // print_vector(next_y_vals_);
}

void PathPlanner::transform_trajectory_to_global_coordinates()
{
    for (size_t i{ego_.prev_path_x_.size()}; i < 50; ++i)
    {
        double cur_x = next_x_vals_.at(i);
        double cur_y = next_y_vals_.at(i);

        // Rotation
        next_x_vals_[i] = cur_x * std::cos(ref_.yaw) - cur_y * std::sin(ref_.yaw);
        next_y_vals_[i] = cur_x * std::sin(ref_.yaw) + cur_y * std::cos(ref_.yaw);

        // translation
        next_x_vals_[i] += ref_.x;
        next_y_vals_[i] += ref_.y;
    }

    // std::cout << "next_x and next_y vals: \n";
    // print_vector(next_x_vals_);
    // print_vector(next_y_vals_);
}
