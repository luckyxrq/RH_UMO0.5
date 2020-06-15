#include "bsp.h"

typedef struct
{
	__IO bool isRunning;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	float angle;
	uint32_t pulse;
}CCWRotationDrawArc;

static CCWRotationDrawArc ccwRotationDrawArc;  /*����ʱ����ת �� ������*/

/*
*********************************************************************************************************
*	�� �� ��: bsp_IsCCWRotationDrawArc
*	����˵��: �Ƿ�����ִ�У�����ʱ����ת �� �����ߣ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsCCWRotationDrawArc(void)
{
	return ccwRotationDrawArc.isRunning;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_CCWRotationDrawArcProc
*	����˵��: ����ʱ����ת �� ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_CCWRotationDrawArcProc(void)
{
	if(!ccwRotationDrawArc.isRunning)
		return;
	
	switch(ccwRotationDrawArc.action)
	{
		case 0: /*��ʱ����ת*/
		{
			ccwRotationDrawArc.angle = REAL_ANGLE();
			bsp_SetMotorSpeed(MotorLeft,-3);
			bsp_SetMotorSpeed(MotorRight,3);
			++ccwRotationDrawArc.action;
		}break;
		
		case 1: /*������*/
		{
			if(ABS(REAL_ANGLE() - bsp_AngleAdd(ccwRotationDrawArc.angle, 90)) <= 10)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,2);
				
				/*��¼��ʼ�����ߵ���ʼ�ǶȺ�ʱ�䣬��ֹһֱ������ѭ��������*/
				ccwRotationDrawArc.delay = xTaskGetTickCount();
				ccwRotationDrawArc.angle = REAL_ANGLE();
				++ccwRotationDrawArc.action;
			}
		}break;
		
		case 2:
		{
			/*���û������������Ǹ�else if������������ײ������Ϊ�ڳ��׮����һ�ߣ���Ҫ����������*/
			if(bsp_CollisionScan() != CollisionNone) 
			{
				ccwRotationDrawArc.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, -3);
				bsp_SetMotorSpeed(MotorRight,-3);
				
				ccwRotationDrawArc.action = 3;
			}
			/*�����ߵĹ����з����Ѿ��ڳ��׮������ֱ�ӽ����м����*/
			else if(INCLINATION_GO_L_0 || INCLINATION_GO_L_1 || INCLINATION_GO_L_2 || INCLINATION_GO_R_0 || INCLINATION_GO_R_1 || INCLINATION_GO_R_2 || ROTATE_CW || ROTATE_CCW) 
			{
				
			}
			/*�����߼�����ת��һȦ�����˳����ģʽ*/
			else if(ABS(REAL_ANGLE() - bsp_AngleAdd(ccwRotationDrawArc.angle, 300)) <= 10 && (xTaskGetTickCount() - ccwRotationDrawArc.delay) >= 2000) 
			{
				
			}
		}break;
		
		case 3:
		{
			if(bsp_GetCurrentBothPulse() - ccwRotationDrawArc.pulse >= _SEARCH_PILE_GO_BACK_PULSE) /*ͨ��̽���֪��֮ǰ�жϵķ�λ�Ƿ��ģ�����˳ʱ����ת����ȥ������*/
			{
				ccwRotationDrawArc.angle = REAL_ANGLE();
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,-3);
				++ccwRotationDrawArc.action;
			}
		}break;
		
		case 4:
		{
			
		}break;
	}
}



