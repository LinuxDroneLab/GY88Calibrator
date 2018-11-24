/*
 * KalmanFilter.cpp
 *
 *  Created on: 23 nov 2018
 *      Author: andrea
 */

#include <KalmanFilter.h>

KalmanFilter::KalmanFilter(float Q, float R, float P, float X, float K) : Q(Q), R(R), P(P), X(X), K(K)
{
}

KalmanFilter::~KalmanFilter()
{
    // TODO Auto-generated destructor stub
}
void KalmanFilter::update() {
  K = (P + Q) / (P + Q + R);
  P = R * K;
}

float KalmanFilter::compute(float newValue) {
  update();
  float r = X + (newValue - X) * K;
  X = r;
  return r;
}



