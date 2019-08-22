/*
* @Author: Udacity
* @Last Modified by:   debasis
* @Last Modified time: 2018-12-16 19:49:51
*/

#ifndef PTG_H
#define PTG_H

#include "vehicle.h"

#include <iostream>
#include <vector>

std::vector<double>
JMT(const std::vector<double>& start,
    const std::vector<double>& end,
    const double T);

bool
close_enough(const std::vector<double>& poly,
             const std::vector<double>& target_poly,
             const double eps = 0.01);

#endif
