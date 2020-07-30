#include "bsp.h"


Kalman kalman;

void KalmanInit(Kalman *k)
{
    k->kGain=0;
    k->p=5;	         /*p��ֵ�������ȡ�����ǲ���Ϊ0��0�Ļ������˲����ˣ�*/
    k->q=0.001;	     /*q�������˲��������ƽ���̶ȣ�qԽСԽƽ��*/
    k->r=0.5;	     /*r���������˲����������ʵ�����ߵ�����̶ȣ�ԽСԽ�ӽ�*/
    k->prevData=0;
	
	/*�û��Զ���*/
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
