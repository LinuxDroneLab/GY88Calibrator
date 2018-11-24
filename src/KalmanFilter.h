/*
 * KalmanFilter.h
 *
 *  Created on: 23 nov 2018
 *      Author: andrea
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

class KalmanFilter
{
private:
    float Q;
    float R;
    float P;
    float X;
    float K;

public:
    KalmanFilter(float Q, float R, float P, float X, float K);
    virtual ~KalmanFilter();
    float compute(float newValue);

private:
    void update();

};

#endif /* KALMANFILTER_H_ */
