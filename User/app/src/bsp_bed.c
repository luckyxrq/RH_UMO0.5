#include "bsp.h"

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

void bsp_BedProc(void)
{
	if(!bed.isRunning)
		return;
	
	switch(bed.action)
	{
		case 0: /* ԭ����ת*/
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 1: /* ת90��*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 2: /*�������ر�*/
		{
				bed.pulse = bsp_GetCurrentBothPulse();
			
				bsp_SetEdgeLeftRight(Dir_left);
				bsp_StartEdgewiseRun();
			
				++bed.action;
		}break;
		
		case 3: /* �������� ����ֹͣ���ر�*/
		{
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
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,-6);
			++bed.action;
		}break;
		
		case 5: /* ת90�� */
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 6: /* ��ǰ�߼��� */
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			++bed.action;

		}break;
		
		case 7: /* ͣ�� ��ʱƽ����  ���ɰ� */
		{
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*0.7)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}

		}break;

		case 8: /* ԭ����ת*/
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 9: /* ת90��*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 10: /* ֱ��  �ȵ����� ���ɰ� */
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 10);
				bsp_SetMotorSpeed(MotorRight,10);
				++bed.action;
			}
		}break;
		
		case 11:
		{
			/* ����������ڵ�*/
			if(bsp_GetInfraRedAdcVoltage(IR7) >= 250)
			{
				++bed.erLangGodCnt;
			}
			
			/* �ж϶������ڵ�����  ���ٻ���*/
			if(bed.erLangGodCnt >= 16)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 2);
				bsp_SetMotorSpeed(MotorRight,2);
				
				++bed.action;
			}
		}
		
		case 12: /* ����һ������ ���������ر�*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetEdgeLeftRight(Dir_right);
				bsp_StartEdgewiseRun();
				
				++bed.action;
			}
		}break;
		
		
		case 13: /* �������� ����ֹͣ���ر�*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*4.2)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 14: /* ֱ��*/
		{
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			++bed.action;
		}break;
		
		case 15: /* �ȴ��������� ֹͣ  ���۴˴����� ǰ������  ����*/
		{
			if(bsp_CliffIsDangerous(CliffMiddle))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 16: /* ���˹̶������� ��ԭ����ת*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 17: /* ת��ֱ��  ׼�������ұ�����*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_45_PULSE)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 18: /* �ȴ��������� ֹͣ  ���۴˴����� ��������  ����*/
		{
			if(bsp_CliffIsDangerous(CliffRight))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 19: /* ���˹̶������� ��ԭ����ת*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 400)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,-6);
				++bed.action;
			}
		}break;
		
		case 20: /* ת��ֱ��  ׼�������������*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 21: /* �ȴ��������� ֹͣ  ���۴˴����� ��������  ����*/
		{
			if(bsp_CliffIsDangerous(CliffLeft))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 22: /* ���˹̶�������*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;

		default: break;
	}
}


