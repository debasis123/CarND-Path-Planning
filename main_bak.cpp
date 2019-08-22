/*
* @Author: Udacity
* @Last Modified by:   debasis123
*/

#include "utility.h"
// #include "vehicle.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
// #include <chrono>
#include <thread>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
static string
hasData(const string& s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// static void get_next_location(std::vector<double>& next_x_vals,
//                               std::vector<double>& next_y_vals,
//                               const SelfDrivingCar& sdc,
//                               const TrackWayPoints& wps) {
//   const double dist_inc = 0.3;
//   const size_t n_points = 50;
//   for(int i = 0; i != n_points; ++i) {
//     const double next_s = sdc.s_ + dist_inc*(i+1);  // i+1 to avoid having i==0 in same place
//     const double next_d = 6; // in the middle of the second lane (4 + 4/2)
//     DoublePair xy = getXYFromFrenet(next_s, next_d, wps.s_, wps.x_, wps.y_);
//     next_x_vals.push_back(xy.first);
//     next_y_vals.push_back(xy.second);
//   }
// }

static void
get_next_location(std::vector<double>& next_x_vals,
                  std::vector<double>& next_y_vals,
                  const SelfDrivingCar& sdc,
                  const TrackWayPoints& wps)
{

  double car_x = sdc.x_, car_y = sdc.y_, car_yaw = sdc.yaw_, car_s = sdc.s_, car_d = sdc.d_;
  std::vector<double> previous_path_x = sdc.prev_path_x_, previous_path_y = sdc.prev_path_y_;
  double end_path_s = sdc.end_path_s_, end_path_d = sdc.end_path_d_;
  vector<vector<double>> sensor_fusion;

  //Car's lane. Stating at middle lane.
  int lane = 1;
  //Reference velocity.
  double ref_vel = 0.0; // mph
  double speed_diff = 0.224;
  double max_accel = 49.5;

  // left over previous path after the simulator has already passed certain points
  // say, it was 50 points, but simulator reports when it is a 3rd point.
  // so prev_size = 47
  int prev_size = previous_path_x.size();

  ////////////////
  // PREDICTION //
  ////////////////

  /***
  The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.
  ***/

  /*
    In this example prediction module we use to find out following
    car ahead is too close, car on the left is too close, and car on the right is too close.
    As explained actual prediction module will be implementd using the apparoach mentioned above, but this highway project
    doesnt need to predict the trajectory of each vehicle as those vehicles trajectory will be on the straight lane.
  */

  if(prev_size > 0) {
    car_s = end_path_s;
  }

  bool car_left= false;
  bool car_right = false;
  bool car_ahead = false;

  for(int i=0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    int check_car_lane = 0;

    /*Currently we assume that we have only three lanes and each lane has 4 meter width. In actual scenarion,
    number of lanes an ddistance between the lanes and total lanes distance can be detected using computer vision
    technologies. We slightly touched in advanced lane findings in term1.
    */
    if(d > 0 && d < 4) {
      check_car_lane = 0;
    } else if(d > 4 && d < 8) {
      check_car_lane = 1;
    } else if(d > 8 and d < 12) {
      check_car_lane = 2;
    }

    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[i][5];

    //This will help to predict the where the vehicle will be in future
    check_car_s += ((double)prev_size*0.02*check_speed);
    if(check_car_lane == lane) {
      //A vehicle is on the same line and check the car is in front of the ego car
      car_ahead |= check_car_s > car_s && (check_car_s - car_s) < 30;

    } else if((check_car_lane - lane) == -1) {
      //A vehicle is on the left lane and check that is in 30 meter range
      car_left |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

    } else if((check_car_lane - lane) == 1) {
      //A vehicle is on the right lane and check that is in 30 meter range
      car_right |= (car_s+30) > check_car_s  && (car_s-30) < check_car_s;

    }
  }

  //As we said, actual prediction module gives the possible trajectories from the current timeline to the future of each vehicle.
  //In this highway exmaple, we will have only one possible trajectory for each vehicle and that is why we are using simple approach as above.
  //In complex situation we may need to use model, data, or hybrid approach for prdiction module

  //BEHAVIOUR
  /***
  The behavioral planning component determines what behavior the vehicle should exhibit at any point in time.
  For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
  ***/
  if(car_ahead) {
    if(!car_left && lane > 0) {
      lane--;
    } else if(!car_right && lane !=2) {
      lane++;
    } else if(!car_left && lane !=2) {
      lane++;
    }else {
      ref_vel -= speed_diff;
    }
  } else if(ref_vel < max_accel){
    ref_vel += speed_diff;
  }
  //In actual case, behaviour planner decides the trajectory based on the cost functions.
  //In this highway example, we may no need to worry about cost functions as we are considering only lane change or reduce speed based on the obstacles.

  //TRAJECTORY
  /***
  Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
  ***/
  vector<double> pts_x;
  vector<double> pts_y;

  //Refrence x,y, and yaw states
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // If previous states are almost empty, use the car as a starting point
  if ( prev_size < 2 ) {
    //Use two points thats makes path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(car_x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(car_y);
  }
  else {
    //Redefine the reference point to previous point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];

    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);

    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y);
  }

  // Setting up target points in the future.
  const double lane_width = 4;
  const double half_lane = lane_width/2;
  DoublePair next_wp0 = getXYFromFrenet(car_s+30, half_lane+lane_width*lane, wps.s_, wps.x_, wps.y_);
  DoublePair next_wp1 = getXYFromFrenet(car_s+60, half_lane+lane_width*lane, wps.s_, wps.x_, wps.y_);
  DoublePair next_wp2 = getXYFromFrenet(car_s+90, half_lane+lane_width*lane, wps.s_, wps.x_, wps.y_);

  pts_x.push_back(next_wp0.first);
  pts_x.push_back(next_wp1.first);
  pts_x.push_back(next_wp2.first);

  pts_y.push_back(next_wp0.second);
  pts_y.push_back(next_wp1.second);
  pts_y.push_back(next_wp2.second);

  // so 5 points so far, 2 refs, and 30, 60, 90 ms

  // Making coordinates to local car coordinates.
  for ( int i = 0; i < pts_x.size(); i++ ) {
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;

    pts_x[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
    pts_y[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
  }

  // Create the spline.
  tk::spline s;
  s.set_points(pts_x, pts_y);  // 5 anchor points

  //For the smooth transition, we are adding previous path points
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate distance y position on 30 m ahead.
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {

    double N = target_dist/(0.02*ref_vel/2.24);  // from mph to mps
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //Rotate back to normal after rotating it earlier
    // go back to global coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

int main()
{
  uWS::Hub h;

  // Waypoint map to read from
  string map_file("../data/highway_map.csv");
  // The max max_track_length value before wrapping around the track back to 0
  const double max_track_length = 6945.554; // in meters

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  ifstream ifs(map_file.c_str(), ifstream::in);
  TrackWayPoints wps(ifs);

  // start at lane 1
  const int lane = 1;
  // have a reference velocity to target
  const double ref_vel = 49.5;  // mph

  h.onMessage([&wps] (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    //////////////////////////////
    // read data from simulator //
    //////////////////////////////
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          SelfDrivingCar sdc(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"],
                             j[1]["previous_path_x"], j[1]["previous_path_y"],
                             j[1]["end_path_s"], j[1]["end_path_d"],
                             j[1]["sensor_fusion"]
                            );

          /////////////////////////////////
          // send data back to simulator //
          /////////////////////////////////
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          get_next_location(next_x_vals, next_y_vals, sdc, wps);

          // assert(next_x_vals.size() == 50 && "wrong vector size");
          // assert(next_y_vals.size() == 50 && "wrong vector size");

          // print_vector(next_x_vals); print_vector(next_y_vals);

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP
  // but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res,
   uWS::HttpRequest req,
   char* data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws,
   int code, char* message, size_t length) {
    ws.close();
  });

  const int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
