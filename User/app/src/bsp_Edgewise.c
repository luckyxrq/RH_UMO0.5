#include "bsp.h"

#define STRAIGHT_SPEED_FAST      12
#define STRAIGHT_SPEED_SLOW      6

#define TURN_RIGHT_SPEED_FAST_L  3
#define TURN_RIGHT_SPEED_FAST_R  1

//
#define TURN_RIGHT_SPEED_SLOW_L  6
#define TURN_RIGHT_SPEED_SLOW_R  5


#define TURN_LEFT_SPEED_FAST_L   1
#define TURN_LEFT_SPEED_FAST_R   3
                     
//					 
#define TURN_LEFT_SPEED_SLOW_L         5
#define TURN_LEFT_SPEED_SLOW_R         6
                                       
#define PIROUETTE_SPEED                6
                                       
#define ROTATE_CW_SPEED_L              5
#define ROTATE_CW_SPEED_R              -5
                                       
#define ROTATE_CCW_SPEED_L             -5
#define ROTATE_CCW_SPEED_R             5
                                       
#define BACKWARD_SPEED                 -6
                                       
/*���Ӻ���20MM��������*/               
#define GO_BACK_PULSE                  (10/(3.14F*70)*1024)
#define COLLISION_STEERING_ANGLE       30.0F

/*�����ض��ٴ���Ϊ����*/
#define POSSIBLE_END      66

typedef struct
{
	volatile bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	uint32_t pulse;
	float angle;
	Collision collision;
	uint32_t possibleEnd;
	uint32_t ErlangGodStartTime ;
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
static float myabs(float val)                ;
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
	edgewiseRun.collision = CollisionNone;
	edgewiseRun.possibleEnd = 0 ;
	edgewiseRun.ErlangGodStartTime = 0 ;
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
	edgewiseRun.collision = CollisionNone;
	edgewiseRun.possibleEnd = 0 ;
	edgewiseRun.ErlangGodStartTime = 0 ;

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
			if(bsp_GetInfraRedAdcVoltage(IR7) >= 1.0F )
			{
				edgewiseRun.ErlangGodStartTime = xTaskGetTickCount() ;
				bsp_EdgewiseRunStraightSlow();
			}
			else
			{
				if(xTaskGetTickCount() - edgewiseRun.ErlangGodStartTime >= 1000)
				{
					bsp_EdgewiseRunStraightFast();
				}
			}
			if(bsp_CollisionScan() != CollisionNone)
			{
				edgewiseRun.action++;
			}
		}break;
		
		case 1:/*�����������ײ����ײ����*/
		{
			float vol = bsp_GetInfraredVoltageRight();
			edgewiseRun.collision = bsp_CollisionScan();
			
			if(edgewiseRun.collision != CollisionNone)
			{
				bsp_GoBackward();
				/*��¼�µ�ǰ�����壬�����˺�ָ�������������룩��ͬʱ��¼�µ�ǰʱ�䣬�������˺ܾû�û֪��*/
				edgewiseRun.pulse = bsp_GetCurrentBothPulse();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
			else if(vol >= 1.5F && vol <=2.0F )
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.possibleEnd = 0 ;
			}
			/*���ҿ����Ĺ����л���Ҫ��⿿����ʱ�䣬��������˺ܾû���û���ҵ���ѹֵ����ô�����ߵ��˾�ͷ*/
			else if(vol < 1.5F)
			{
				bsp_EdgewiseTurnRightSlow();
				if(vol < 0.2F)
				{
					
					if(edgewiseRun.possibleEnd++ >= POSSIBLE_END)
					{
						edgewiseRun.possibleEnd = 0 ;
						edgewiseRun.action = 4 ;
					}
				}
			}
			else if(vol > 2.0F)
			{
				bsp_EdgewiseTurnLeftSlow();
				edgewiseRun.possibleEnd = 0 ;
			}

		}break;
		
		case 2:/*����������ԭ����ת�����Ҷ���*/
		{
			if((xTaskGetTickCount() - edgewiseRun.delay >= 2000) || (bsp_GetCurrentBothPulse()-edgewiseRun.pulse)>=GO_BACK_PULSE)
			{
				bsp_RotateCCW();
				/*��ȡ�ǶȺ�ʱ�䣬ת���̶��Ƕȣ�ʱ��̫�û�ûת�������쳣*/
				edgewiseRun.angle = bsp_AngleRead();
				edgewiseRun.delay = xTaskGetTickCount();
				edgewiseRun.action++;
			}
		}break;
		
		case 3:/*��תһ���������ֱ�У��ص�״̬1*/
		{
			float val = bsp_GetInfraredVoltageRight();
			if(myabs(bsp_AngleAdd(edgewiseRun.angle ,20) - (bsp_AngleRead())) <= 2.0F ||
				(val>=1.0F && val<=3.3F && myabs(bsp_AngleRead()-edgewiseRun.angle)>=10.0F ))
			{
				bsp_EdgewiseRunStraightSlow();
				edgewiseRun.action = 1;
			}

		}break;

		case 4: /*��ȫ��ʧ�������ߣ������ת�Ƕ�*/
		{
			edgewiseRun.angle = bsp_AngleRead();
			edgewiseRun.delay = xTaskGetTickCount();
			
			
			bsp_PirouetteCW();
			edgewiseRun.action++;
		}break;
		
		case 5:
		{
			float vol = bsp_GetInfraredVoltageRight();
			
			
			/*�ж�����ת��̫����*/
			if((xTaskGetTickCount() - edgewiseRun.delay)>= 3000 && 
				myabs(bsp_AngleAdd(edgewiseRun.angle ,360) - (bsp_AngleRead())) <= 10.0F)
			{
				edgewiseRun.action = 0 ;
			}
			

			if(bsp_CollisionScan()!=CollisionNone || (vol >= 1.2F && vol <=3.3F ))
			{
				edgewiseRun.action = 1 ;
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
#if 1
	
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);

#else
	
/*
	linearVelocity   ���ٶȣ�����/s��
	angularVelocity  ���ٶȣ���/s��
	r                ���˾��룬Ҳ��ת��뾶�����ף�
*/
	
	/*���������������������ٶȺͽ��ٶ�*/
	double r = 15;
	double linearVelocity = 20;
	double angularVelocity = Deg2Rad(linearVelocity / r);

	/*������ٶȣ���λMM/S */
	int16_t leftVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 - Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
	int16_t rightVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 + Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
	
	/*�����ٶȣ���λMM/S */
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
	
#endif

}



static float myabs(float val)
{
	if(val < 0)
	{
		val = - val;
	}
	
	return val;
}


