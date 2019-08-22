/*
* @Author: Udacity
* @Last Modified by:   debasis
* @Last Modified time: 2018-12-16 19:49:51
*/

#ifndef COST_H
#define COST_H

#include "vehicle.h"

double
calculate_cost(const Vehicle& vehicle,
               const std::map<VehicleId, Trajectory>& predictions,
               const Trajectory& trajectory);

#endif
