# CarND-Path-Planning-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Goals
In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946 m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### Map of the highway
The map of the highway is in `data/highway_map.txt`. Each waypoint in the list contains `[x,y,s,dx,dy]` values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the Frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in Frenet coordinates

["d"] The car's d position in Frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's Frenet s value

["end_path_d"] The previous list's last point's Frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in Frenet coordinates, car's d position in Frenet coordinates].



## Reflections.


The goal of this implementation is to drive the ego car on the fastest possible lane, close to but not exceeding the speed limit of 50 mph, max acceleration of 10 m/s^2, and max jerk of 10 m/s^3.

The helper functions are implemented in [helpers.cpp](./src/helpers.cpp) and the simulation data read from the json file is stored into `Vehicle` and `Ego` data structures, implemented in [simulation_data.h](./src/simulation_data.h) and [simulation_data.cpp](./simulation_data.cpp) files. [parameters.h](./src/parameters.h) contains the parameters and some hyperparameters for my implementation.

The implementation starts at [main.cpp](./src/main.cpp). We read the highway map from [highway_map.csv](./data/highway_map.csv) and store in a data structure `HighwayMap`. Then we continue reading the simulation data in json format and run the path planning code on that data to generate the next trajectory of the ego car, which eventually is fed back into simulator at every step. What follows is the implementation for this `path planning` part.


### Path Planner
Path Planning starts at [path_planning.cpp](./src/path_planning.cpp). Main class is `PathPlanner` and the main function that does the job is `process()`.

`main()` function calls `PathPlanner` for all its work regarding the path planning and sends the simulator back `next_x_vals` and `next_y_vals` (ego trajectory) that is updated by PathPlanner.

Note that we only predict the next step (0.02 sec) of the ego vehicle **after** the previous trajectory. First, we place the ego vehicle at the end of the previous trajectory. We do this so as to reuse the trajectory from previous simulation. This provides a smoother continuation of the trajectory for the ego.

The overall implementation (each simulation step) is divided into 5 steps.

#### 1. Prediction
Implemented in file [prediction.cpp](./src/prediction.cpp). Main class is `Prediction` and the main function that does the job is `process()`.

`PathPlanner` calls prediction to predict the agent vehicles around it at the end of the previous trajectory. Note that each point of the trajectory is differed by 0.02 sec. We assume `constant velocity` motion model for the agent cars and thus has only linear trajectory (in terms of s value in Frenet).

#### 2. Behavior Planning
Implemented in file [behavior_planning.cpp](./src/behavior_planning.cpp). Main class is `BehaviorPlanner` and the main function that does the job is `process()`.

Next, we compute the possible states of the ego vehicle based on the agents around it. We use only "Keep Lane" (KL), "Lane Change Left" (LCL), and "Lane Change Right" (LCR) states in the FSM.  The algorithm (`process()` function) for it is the following:
1. For each possible state of ego,
    - Compute all possible trajectories (next lane) of the ego (calls `process()` of `TrajectoryGenerator`)
    - For each possible trajectory, compute the cost of the trajectory as below. (calls `calculate_cost()` of `CostCalculator`)
2. Return the trajectory and the state with the minimum cost of the associated trajectory.

#### 3. Trajectory Generation
Implemented in file [trajectory_generation.cpp](./src/trajectory_generation.cpp). Main class is `TrajectoryGenerator` and the main function that does the job is `process()`.

This module computes the next lane based on whether there is vehicle on the front, left or right. The ego changes lane to be on faster lane and reduces speed if there is a vehicle in the front and it cannot change the lane. If there is no obstacle on its lane, it continues to increase speed to drive close to 50 mph. The related max acceleration or deceleration parameters are described in [parameters.h](./src/parameters.h).

#### 4. Cost of Trajectory
Implemented in file [cost_functions.cpp](./src/cost_functions.cpp). Main class is `CostCalculator` and the main function that does the job is `calculate_cost()`.

This module computes the cost of a given trajectory coming out of trajectory computation. We compute two types of weighted costs, taken from the course tutorial.
1. inefficiency_cost: Cost becomes higher for trajectories with intended next lane and final lane that have traffic slower than vehicle's target speed (50 mph).
2. goal_distance_cost: Cost based on distance of intended next lane and final lane of trajectory.

#### 5. Spline Trajectory
Implemented in file [path_planning.cpp](./src/path_planning.cpp).

The `process()` function in `PathPlanner` does the following for the smoother trajectory generation.
1. First create 6 *anchor points*
     - first two points from previous trajectory, if exists, or interpolated points, otherwise (implemented by `add_past_anchor_waypoints()`). They are already in global x-y coordinates.
     - then we use the lane from behavior modeling and trajectory generation to create four more anchor points at 30, 60, 90, and 120 m distances from ego's position (s values on Frenet)
     - convert them to global x-y coordinates using `getXYFromFrenet()` from [helpers.h](./src/helpers.h)
2. Transform these 6 anchor points from global coordinates to to car's own coordinates and use them as the seed for the spline (`transform_trajectory_to_car_coordinates()`)
3. Generate enough spline points to make the trajectory size consisting of 50 points to be fed to the simulator (`create_spline_trajectory()`). Convert these new spline points back to global coordinates (`transform_trajectory_to_global_coordinates()`), as the previous trajectory points are in global coordinates and so is the expectation from the simulator.

