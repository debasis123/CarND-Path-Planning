/*
* @Author: Udacity
* @Last Modified by:   debasis
* @Last Modified time: 2018-12-16 19:49:51
*/

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <algorithm>
#include <random>
#include <vector>
#include <map>
#include <string>
#include <cassert>

/*
data from simulator simulating sensor_fusion module about traffic on road
 */
class SimCar {
public:
  // unique identifier for that car
  size_t id_;
  // location in global map coordinates
  double x_, y_;
  // velocity components in global map coordinates, in mps
  double vx_, vy_;
  // Frenet coordinates for that car
  double s_, d_;

  SimCar(const std::vector<double>& v)
    : id_(v.at(0))
    , x_(v.at(1)), y_(v.at(2)), vx_(v.at(3)), vy_(v.at(4)), s_(v.at(5)), d_(v.at(6))
  {}
  ~SimCar() {}
};

/*
sensor_fusion data about self driving car and its surrounding traffic
 */
class SimSdc {
public:
  // Main car's localization Data (No Noise)
  double x_, y_; // The car's x-y position in global map X_Y coordinates
  double s_, d_; // The car's s-d position in Frenet coordinates
  double yaw_; // The car's yaw angle in the map
  double speed_; // The car's speed in MPH
  std::vector<double> prev_path_x_, prev_path_y_; // The list of x-y points previously given to the simulator,
                                                  // but with processed points removed
  double end_path_s_, end_path_d_; // The previous path's last point's Frenet s-d values
  // Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
  std::vector<SimCar> sensor_fusion_;

  SimSdc(const double x,
         const double y,
         const double s,
         const double d,
         const double yaw,
         const double speed,
         const std::vector<double>& px,
         const std::vector<double>& py,
         const double eps,
         const double epd,
         const std::vector<std::vector<double>>& sf)
    : x_(x)
    , y_(y)
    , s_(s)
    , d_(d)
    , yaw_(yaw)
    , speed_(speed)
    , prev_path_x_(px)
    , prev_path_y_(py)
    , end_path_s_(eps)
    , end_path_d_(epd)
  {
    std::for_each(sf.begin(), sf.end(),
      [&](const std::vector<double> carInfo) {
        sensor_fusion_.push_back(SimCar(carInfo));
      });
    assert(sensor_fusion_.size() == sf.size() && "size should be same");
  }
  ~SimSdc() {}
};


class Vehicle;

using State = std::string;
using Trajectory = std::vector<Vehicle>;
using States = std::vector<State>;
using VehicleId = int;
using Predictions = std::map<VehicleId, Trajectory>;
using LaneId = int;
using Lanes = std::vector<LaneId>;

/*
This is our self driving car that combines data from SimSdc, Car and it own
 */
class Vehicle {
public:
  std::map<State, int> lane_direction = { { "PLCL", 1 },
                                          { "LCL", 1 },
                                          { "LCR", -1 },
                                          { "PLCR", -1 } };

  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };

  int L = 1;
  int preferred_buffer = 6; // safe_distance between vehicles
  LaneId lane;
  int s;
  double v; // velocity
  double a; // acceleration
  double target_speed;  // speed_limit - buffer_speed
  int n_lanes_available;
  double max_acceleration;
  int goal_lane;
  int goal_s;
  State state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(const SimSdc& simsdc);
  Vehicle(int lane, double s, double v, double a, State state = "CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  Trajectory    generate_predictions(int horizon = 2);
  Trajectory    choose_next_state(Predictions predictions);
  void          realize_next_state(Trajectory trajectory);
  void          increment(int dt = 1);
  void          configure(Lanes road_data);

private:
  States        successor_states();
  Trajectory    generate_trajectory(State state, Predictions predictions);
  Trajectory    constant_speed_trajectory();
  Trajectory    keep_lane_trajectory(Predictions predictions);
  Trajectory    lane_change_trajectory(State state, Predictions predictions);
  Trajectory    prep_lane_change_trajectory(State state, Predictions predictions);
  double        position_at(int t);
  std::vector<double>    get_kinematics(Predictions predictions, LaneId lane);
  bool          is_vehicle_behind(Predictions predictions, LaneId lane, Vehicle& rVehicle);
  bool          is_vehicle_ahead(Predictions predictions, LaneId lane, Vehicle& fVehicle);
};

#endif
