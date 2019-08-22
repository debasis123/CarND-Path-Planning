/*
* @Author: Udacity
* @Last Modified by:   debasis123
* @Last Modified time: 2018-12-26 10:39:03
*/

#include "PTG.h"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// TODO - complete this function
vector<double>
JMT(const vector<double>& start, const vector<double>& end, const double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS
  start - the vehicle's start location is given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the
  polynomial s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 *
  t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  double C0 = start.at(0);
  double C1 = start.at(1);
  double C2 = start.at(2);

  std::vector<double> result {C0, C1, C2/2.0};

  double T2 = T*T;
  double T3 = T*T2;
  double T4 = T2*T2;
  double T5 = T*T4;

  MatrixXd A = MatrixXd(3, 3);
  A << T3,    T4,     T5,
       3*T2,  4*T3,   5*T4,
       6*T,   12*T2,  20*T3;

  VectorXd B = VectorXd(3);
  B << end.at(0) - (C0+C1*T+C2/2*T2),
       end.at(1) - (C1+C2*T),
       end.at(2) - C2;

  VectorXd alpha = A.inverse() * B;
  for (int i = 0; i < alpha.size(); ++i) {
      result.push_back(alpha.data()[i]);
  }
  return result;
}

bool
close_enough(const vector<double>& poly,
             const vector<double>& target_poly,
             const double eps)
{
  if (poly.size() != target_poly.size()) {
    cerr << "your solution didn't have the correct number of terms" << endl;
    return false;
  }
  for (int i = 0; i < poly.size(); ++i) {
    double diff = poly[i] - target_poly[i];
    if (abs(diff) > eps) {
      cerr << "at least one of your terms differed from target by more than "
      << eps << endl;
      return false;
    }
  }
  return true;
}

struct test_case {
  vector<double> start;
  vector<double> end;
  double T;
};

const vector<vector<double>> answers = {
  { 0.0, 10.0, 0.0, 0.0, 0.0, 0.0 },
  { 0.0, 10.0, 0.0, 0.0, -0.625, 0.3125 },
  { 5.0, 10.0, 1.0, -3.0, 0.64, -0.0432 }
};

int main()
{
  // create test cases

  vector<test_case> tc;

  test_case tc1;
  tc1.start = {0,10,0};
  tc1.end = {10,10,0};
  tc1.T = 1;
  tc.push_back(tc1);

  test_case tc2;
  tc2.start = {0,10,0};
  tc2.end = {20,15,20};
  tc2.T = 2;
  tc.push_back(tc2);

  test_case tc3;
  tc3.start = {5,10,2};
  tc3.end = {-30,-20,-4};
  tc3.T = 5;
  tc.push_back(tc3);

  bool total_correct = true;
  for(int i = 0; i < tc.size(); ++i) {
    const vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    const bool correct = close_enough(jmt,answers[i]);
    total_correct &= correct;
  }
  if(!total_correct) {
    cerr << "Try again!" << endl;
  } else {
    cout << "Nice work!" << endl;
  }

  return 0;
}
