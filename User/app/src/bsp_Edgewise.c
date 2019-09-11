#include "bsp.h"

#define STRAIGHT_SPEED_FAST      3
#define STRAIGHT_SPEED_SLOW      3

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

//
#define TURN_RIGHT_SPEED_SLOW_L  4
#define TURN_RIGHT_SPEED_SLOW_R  2


#define TURN_LEFT_SPEED_FAST_L   1
#define TURN_LEFT_SPEED_FAST_R   3
                     
//					 
#define TURN_LEFT_SPEED_SLOW_L   2
#define TURN_LEFT_SPEED_SLOW_R   4

#define PIROUETTE_SPEED          1

#define ROTATE_CW_SPEED_L        2
#define ROTATE_CW_SPEED_R        -2

#define ROTATE_CCW_SPEED_L       -2
#define ROTATE_CCW_SPEED_R       2

#define BACKWARD_SPEED           -6


typedef struct
{
	volatile bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	
}EdgewiseRun;

static EdgewiseRun edgewiseRun;
static void bsp_EdgewiseRunStraightFast(void);
static void bsp_EdgewiseRunStraightSlow(void);
static void bsp_EdgewiseTurnRightFast(void)  ;
static void bsp_EdgewiseTurnRightSlow(void)  ;
static void bsp_EdgewiseTurnLeftFast(void)   ;
static void bsp_EdgewiseTurnLeftSlow(void)   ;
static void bsp_PirouetteCW(void)            ;
static void bsp_PirouetteCCW(void)           ;
static void bsp_RotateCW(void)               ;
static void bsp_RotateCCW(void)              ;
static void bsp_GoBackward(void)             ;

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartEdgewiseRun
*	����˵��: �����ر�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartEdgewiseRun(void)
{
	edgewiseRun.action = 0 ;
	edgewiseRun.delay = 0 ;
	edgewiseRun.isRunning = true;
	
	/*��������������*/
	UNUSED(bsp_EdgewiseRunStraightFast) ;
	UNUSED(bsp_EdgewiseRunStraightSlow) ;
	UNUSED(bsp_EdgewiseTurnRightFast)   ;
	UNUSED(bsp_EdgewiseTurnRightSlow)   ;
	UNUSED(bsp_EdgewiseTurnLeftFast)    ;
	UNUSED(bsp_EdgewiseTurnLeftSlow)    ;
	UNUSED(bsp_PirouetteCW)             ;
	UNUSED(bsp_PirouetteCCW)            ;
	UNUSED(bsp_GoBackward)              ;
	UNUSED(bsp_RotateCW)                ;
	UNUSED(bsp_RotateCCW)               ;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopEdgewiseRun
*	����˵��: �ر��ر�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopEdgewiseRun(void)
{
	edgewiseRun.isRunning = false;
	edgewiseRun.action = 0 ;
	edgewiseRun.delay = 0 ;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEdgewiseRun
*	����˵��: ��ʼ���ر�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_EdgewiseRun(void)
{
	if(!edgewiseRun.isRunning)
		return ;
	
	switch(edgewiseRun.action)
	{
		case 0:/*�����ر�ģʽ������ֱ��*/
		{
			bsp_EdgewiseRunStraightSlow();
			edgewiseRun.action++;
		}break;
		
		case 1:/*�����������ײ����ײ����*/
		{
			Collision ret = bsp_CollisionScan();
			float vol = bsp_GetInfraredVoltageRight();
				
			if(ret != CollisionNone)
			{
				bsp_GoBackward();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
			else if(vol >= 1.2F && vol <=2.5F )
			{
				bsp_EdgewiseRunStraightSlow();
			}
			else if(vol < 1.2F)
			{
				bsp_EdgewiseTurnRightSlow();
			}
			else if(vol > 2.5F)
			{
				bsp_EdgewiseTurnLeftSlow();
			}
		}break;
		
		case 2:/*����������ԭ����ת�����Ҷ���*/
		{
			if(xTaskGetTickCount() - edgewiseRun.delay >= 800)
			{
				bsp_RotateCCW();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
		}break;
		
		case 3:/*��תһ���������ֱ�У��ص�״̬1*/
		{
			if(xTaskGetTickCount() - edgewiseRun.delay >= 800)
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 1;
			}
		}break;

		
		
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseRunStraightFast
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseRunStraightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_FAST);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_FAST);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseRunStraightSlow
*	����˵��: ����ֱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseRunStraightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, STRAIGHT_SPEED_SLOW);
	bsp_SetMotorSpeed(MotorRight,STRAIGHT_SPEED_SLOW);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseTurnRightFast
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnRightFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseTurnRightSlow
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnRightSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_RIGHT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_RIGHT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseTurnLeftFast
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnLeftFast(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_FAST_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_FAST_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EdgewiseTurnLeftSlow
*	����˵��: ������ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EdgewiseTurnLeftSlow(void)
{
	bsp_SetMotorSpeed(MotorLeft, TURN_LEFT_SPEED_SLOW_L);
	bsp_SetMotorSpeed(MotorRight,TURN_LEFT_SPEED_SLOW_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PirouetteCW
*	����˵��: ԭ��˳ʱ����ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PirouetteCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, PIROUETTE_SPEED);
	bsp_SetMotorSpeed(MotorRight,0);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PirouetteCCW
*	����˵��: ԭ����ʱ����ת
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PirouetteCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,PIROUETTE_SPEED);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GoBackward
*	����˵��: �����ϰ���󣬿��ٵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_GoBackward(void)
{
	bsp_SetMotorSpeed(MotorLeft, BACKWARD_SPEED);
	bsp_SetMotorSpeed(MotorRight,BACKWARD_SPEED);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_RotateCW
*	����˵��: ԭ����ת�������ֶ�����˳ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_RotateCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CW_SPEED_R);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_RotateCCW
*	����˵��: ԭ����ת�������ֶ�������ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_RotateCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
}
