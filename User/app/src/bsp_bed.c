#include "bsp.h"

#define RTT_BED_EN  0

#if RTT_BED_EN
#define RTT_BED(format, ...) SEGGER_RTT_printf (0,format, ##__VA_ARGS__)
#else
#define RTT_BED(format, ...) {}
#endif	


typedef struct
{
	bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	
	uint32_t pulse;
	uint32_t erLangGodCnt;
}Bed;

static Bed bed;




void bsp_BedStart(void)
{
	bed.action = 0 ;
	bed.delay = 0 ;
	bed.pulse = 0 ;
	bed.erLangGodCnt = 0 ;
	
	bed.isRunning = true;
}

void bsp_BedStop(void)
{	
	bed.isRunning = false;
	
	bed.action = 0 ;
	bed.delay = 0 ;
	bed.pulse = 0 ;
	bed.erLangGodCnt = 0 ;
}

bool bsp_IsBedProcRunning(void)
{
	return bed.isRunning;
}



void bsp_BedProc(void)
{
	if(!bed.isRunning)
		return;
	
	switch(bed.action)
	{
		case 0: /* ԭ����ת*/
		{
			RTT_BED("bed %d\r\n",0);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 1: /* ת90��*/
		{
			RTT_BED("bed %d\r\n",1);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 2: /*�������ر�*/
		{
			RTT_BED("bed %d\r\n",2);
			
			bed.pulse = bsp_GetCurrentBothPulse();
		
			bsp_SetEdgeLeftRight(Dir_left);
			bsp_StartEdgewiseRun();
		
			++bed.action;
		}break;
		
		case 3: /* �������� ����ֹͣ���ر�*/
		{
			RTT_BED("bed %d\r\n",3);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*3)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 4: /* ԭ����ת */
		{
			RTT_BED("bed %d\r\n",4);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,-6);
			++bed.action;
		}break;
		
		case 5: /* ת90�� */
		{
			RTT_BED("bed %d\r\n",5);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 6: /* ��ǰ�߼��� */
		{
			RTT_BED("bed %d\r\n",6);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			++bed.action;

		}break;
		
		case 7: /* ͣ�� ��ʱƽ����  ���ɰ� */
		{
			RTT_BED("bed %d\r\n",7);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*0.7)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}

		}break;

		case 8: /* ԭ����ת*/
		{
			RTT_BED("bed %d\r\n",8);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 9: /* ת90��*/
		{
			RTT_BED("bed %d\r\n",9);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 10: /* ֱ��  �ȵ����� ���ɰ� */
		{
			RTT_BED("bed %d\r\n",10);
			
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			bed.erLangGodCnt = 0 ;
			
			++bed.action;
		}break;
		
		case 11:
		{
			RTT_BED("bed %d\r\n",11);
			
			/* ����������ڵ�*/
			if(bsp_GetInfraRedAdcVoltage(IR7) >= bsp_GetParaErLangShen())
			{
				RTT("IR7:%d\r\n",(int)bsp_GetInfraRedAdcVoltage(IR7));
				
				
				++bed.erLangGodCnt;
				
				/* �ж϶������ڵ�����  ���ٻ���*/
				if(bed.erLangGodCnt >= 10)
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					bed.delay = xTaskGetTickCount();
					
					++bed.action;
				}
			}
		}break;
		
		case 12:
		{
			RTT_BED("bed %d\r\n",12);
			
			/* �ڶ��ɰ��ӵ��¼�������*/
			if(xTaskGetTickCount() - bed.delay >= 200)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,3);

				++bed.action;
			}
		}break;

		case 13: /* ����һ������ ���������ر�*/
		{
			RTT_BED("bed %d\r\n",13);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700 * 0.6)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetEdgeLeftRight(Dir_right);
				bsp_StartEdgewiseRun();
				
				++bed.action;
			}
		}break;
		
		case 14: /* �������� ����ֹͣ���ر�*/
		{
			RTT_BED("bed %d\r\n",14);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*4.2)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 15: /* ֱ��*/
		{
			RTT_BED("bed %d\r\n",15);
			
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			++bed.action;
		}break;
		
		case 16: /* �ȴ��������� ֹͣ  ���۴˴����� ǰ������  ����*/
		{
			RTT_BED("bed %d\r\n",16);
			
			if(bsp_CliffIsDangerous(CliffMiddle))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 17: /* ���˹̶������� ��ԭ����ת*/
		{
			RTT_BED("bed %d\r\n",17);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 18: /* ת��ֱ��  ׼�������ұ�����*/
		{
			RTT_BED("bed %d\r\n",18);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_45_PULSE)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 19: /* �ȴ��������� ֹͣ  ���۴˴����� ��������  ����*/
		{
			RTT_BED("bed %d\r\n",19);
			
			if(bsp_CliffIsDangerous(CliffRight))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 20: /* ���˹̶������� ��ԭ����ת*/
		{
			RTT_BED("bed %d\r\n",20);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 400)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,-6);
				++bed.action;
			}
		}break;
		
		case 21: /* ת��ֱ��  ׼�������������*/
		{
			RTT_BED("bed %d\r\n",21);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 22: /* �ȴ��������� ֹͣ  ���۴˴����� ��������  ����*/
		{
			RTT_BED("bed %d\r\n",22);
			
			if(bsp_CliffIsDangerous(CliffLeft))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 23: /* ���˹̶�������*/
		{
			RTT_BED("bed %d\r\n",23);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1200)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 24: /* ԭ����ת  ��ʼ�Ƕ��� 0   ��ת��180��  ��֤������*/
		{
			RTT_BED("bed %d\r\n",24);
			
			bsp_SetMotorSpeed(MotorLeft, -3);
			bsp_SetMotorSpeed(MotorRight, 3);
			++bed.action;	
			
		}break;
		
		case 25: /* ͣ��180 ��*/
		{
			RTT_BED("bed %d\r\n",25);
			
			float yaw = ABS(bsp_AngleRead());	
			
			if( yaw >= 170 && yaw <= 180 )
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				bed.delay = xTaskGetTickCount();
				
				++bed.action;
			}
		}break;
		
		
		case 26: /*ͣ����MS*/
		{
			RTT_BED("bed %d\r\n",26);
			
			if(xTaskGetTickCount() - bed.delay >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 8);
				bsp_SetMotorSpeed(MotorRight,8);
				
				++bed.action;
			}
		}break;
		
		case 27:
		{

			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700)
			{

				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				++bed.action;
			}
		}break;

		case 28: /*ͣ����MS*/
		{
			/*�رյ��*/
			bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
			bsp_StopVacuum();
			
			/*LED*/
			bsp_OpenThreeWhileLed();
			
			bsp_BedStop();
			
			/*���س��*/
			bsp_PutKey(KEY_LONG_CHARGE);
		}break;
		
		

		default: break;
	}
}



// LED_LOGO_CLEAN,LED_LOGO_POWER,LED_LOGO_CHARGE,


#define LED_ARR_SZIE  3

static LED_SN ledArr[LED_ARR_SZIE] = {LED_COLOR_YELLOW,LED_COLOR_GREEN,LED_COLOR_RED};


void bsp_LedBedTurns(void)
{
	static uint8_t toggleIndex = 0 ;
	
	bsp_LedOff(LED_LOGO_CLEAN);
	bsp_LedOff(LED_LOGO_POWER);
	bsp_LedOff(LED_LOGO_CHARGE);
	
	bsp_LedToggle(ledArr[toggleIndex % LED_ARR_SZIE]);
	
	if(++toggleIndex >= LED_ARR_SZIE)
	{
		toggleIndex = 0;
	}
}
