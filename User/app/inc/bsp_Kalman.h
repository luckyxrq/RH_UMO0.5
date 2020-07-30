#ifndef __BSP_KALMAN_H
#define __BSP_KALMAN_H


typedef struct
{
    double prevData;
    double p,q,r,kGain;
}Kalman;


void KalmanInit(Kalman *k);
double KalmanFilter(Kalman *k,double data);

#endif



