#include "bsp.h"


Kalman kalman;

void KalmanInit(Kalman *k)
{
    k->kGain=0;
    k->p=5;	         /*p初值可以随便取，但是不能为0（0的话最优滤波器了）*/
    k->q=0.001;	     /*q参数调滤波后的曲线平滑程度，q越小越平滑*/
    k->r=0.5;	     /*r参数调整滤波后的曲线与实测曲线的相近程度，越小越接近*/
    k->prevData=0;
	
	/*用户自定义*/
//	k->p=5;
//	k->q=0.01;
//	k->r=50;
}

double KalmanFilter(Kalman *k,double data)
{
    k->p=k->p+k->q;
    k->kGain=k->p/(k->p+k->r);
    data=k->prevData+k->kGain*(data-k->prevData);
    k->p=(1-k->kGain*k->p);
    k->prevData=data;
    
    return data;
}
