/*
 * @Author: debasis
 * @Last Modified by:   debasis
 */

#ifndef PREDICTION_H
#define PREDICTION_H

#include "simulation_data.h"

/**
 * \brief Predicts the behavior of the moving objects around the ego car
 */
class Predictor
{
public:
    Predictor(Ego& ego, Predictions& predictions);

    void process() const;

private:
    Ego& ego_;
    Predictions& predictions_;
};

#endif // PREDICTION_H
