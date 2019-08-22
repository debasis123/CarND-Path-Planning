/*
* @Author: debasis123
* @Last Modified by:   debasis123
* @Last Modified time: 2018-12-18 20:14:30
*/

#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

using DoublePair = std::pair<double, double>;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(const double x) { return x * pi() / 180; }
double rad2deg(const double x) { return x * 180 / pi(); }

double
distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

template<typename T>
void
print_vector(const std::vector<T>& v)
{
  std::for_each(v.begin(), v.end(),
    [&](const T& t) {
      std::cout << t << " ";
    } );
  std::cout << std::endl;
}

int
ClosestWaypoint(const double x,
                const double y,
                const std::vector<double>& maps_x,
                const std::vector<double>& maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); ++i) {
    double dist = distance(x, y, maps_x.at(i), maps_y.at(i));
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int
NextWaypoint(const double x,
             const double y,
             const double theta,
             const std::vector<double>& maps_x,
             const std::vector<double>& maps_y)
{
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  const double map_x = maps_x.at(closestWaypoint);
  const double map_y = maps_y.at(closestWaypoint);

  const double heading = atan2((map_y-y), (map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
DoublePair
getFrenetFromXY(const double x,
                const double y,
                const double theta,
                const std::vector<double>& maps_x,
                const std::vector<double>& maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x.at(next_wp) - maps_x.at(prev_wp);
  double n_y = maps_y.at(next_wp) - maps_y.at(prev_wp);
  double x_x = x - maps_x.at(prev_wp);
  double x_y = y - maps_y.at(prev_wp);

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x.at(prev_wp);
  double center_y = 2000 - maps_y.at(prev_wp);
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x.at(i), maps_y.at(i),
                         maps_x.at(i + 1), maps_y.at(i + 1));
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
DoublePair
getXYFromFrenet(const double s,
                const double d,
                const std::vector<double>& maps_s,
                const std::vector<double>& maps_x,
                const std::vector<double>& maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1]
        && (prev_wp < static_cast<int>(maps_s.size()-1) )) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1) % maps_x.size();

  double heading = atan2((maps_y.at(wp2) - maps_y.at(prev_wp)),
                         (maps_x.at(wp2) - maps_x.at(prev_wp)));
  // the x,y,s along the segment
  double seg_s = s - maps_s[prev_wp];

  double seg_x = maps_x.at(prev_wp) + seg_s*cos(heading);
  double seg_y = maps_y.at(prev_wp) + seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x, y};
}

class TrackWayPoints {
public:
  // in global map X-Y coordinates
  std::vector<double> x_, y_;
  // in Frenet coordinates
  std::vector<double> s_, dx_, dy_; // d split up into the global x and y components

  TrackWayPoints(std::ifstream& ifs) {
    std::string line;
    while (std::getline(ifs, line)) {
      std::istringstream iss(line);
      double x;   iss >> x;   x_.push_back(x);
      double y;   iss >> y;   y_.push_back(y);
      double s;   iss >> s;   s_.push_back(s);
      double d_x; iss >> d_x; dx_.push_back(d_x);
      double d_y; iss >> d_y; dy_.push_back(d_y);
    }
  }
  ~TrackWayPoints() {}
};

#endif
