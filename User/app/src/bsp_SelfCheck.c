#include "bsp.h"

#define SELF_CHECK_RX_CH1()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
#define SELF_CHECK_RX_CH2()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
#define SELF_CHECK_RX_CH3()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define SELF_CHECK_RX_CH4()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)


#define SELF_CHECK_KEY_CLEAN()       GPIO_ReadInputDataBit(GPIO_PORT_K2,GPIO_PIN_K2)
#define SELF_CHECK_KEY_POWER()       GPIO_ReadInputDataBit(GPIO_PORT_K3,GPIO_PIN_K3)
#define SELF_CHECK_KEY_CHARGE()      GPIO_ReadInputDataBit(GPIO_PORT_K1,GPIO_PIN_K1)

/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*数据部分开始*/
	
	__IO bool isRunning;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	/*下面的全部是BOOL类型*/
	uint8_t b_batteryV;
	uint8_t b_irCommunicate;
	uint8_t b_irRX_1;
	uint8_t b_irRX_2;
	uint8_t b_irRX_3;
	uint8_t b_irRX_4;
	uint8_t b_irRead;
	uint8_t b_cliffL;
	uint8_t b_cliffM;
	uint8_t b_cliffR;
	uint8_t b_wheelL_I;
	uint8_t b_wheelR_I;
	uint8_t b_wheelL_Speed;
	uint8_t b_wheelR_Speed;
	uint8_t b_offsiteL;
	uint8_t b_offsiteR;
	
	uint8_t b_vacuum_I;
	uint8_t b_rollBrush_I;
	uint8_t b_sideBrush_I;
	uint8_t b_whole_I;
	
	uint8_t b_angle;
	
	uint8_t b_touch_1;
	uint8_t b_touch_2;
	uint8_t b_touch_3;
	
	uint8_t b_collision_1;
	uint8_t b_collision_2;

	uint8_t b_hall;
	
	uint8_t isAllOK;
	uint8_t isChkAllComplete;
	
	/*数据部分结束*/
	
	uint16_t crc16;
}SelfCheck;
#pragma pack()


static SelfCheck selfCheck;


void bsp_StartSelfCheck(void)
{
	/*将所有BOOL数据全部清零*/
	memset(&selfCheck, 0 , sizeof(selfCheck));
	
	selfCheck.action = 0 ;
	selfCheck.delay = 0 ;
	selfCheck.isChkAllComplete = 0;
	selfCheck.isAllOK = 0 ;
	selfCheck.isRunning = true;
}

void bsp_StopSelfCheck(void)
{
	selfCheck.isRunning = false;
	
	/*将所有BOOL数据全部清零*/
	memset(&selfCheck, 0 , sizeof(selfCheck));
	
	selfCheck.action = 0 ;
	selfCheck.delay = 0 ;
	selfCheck.isChkAllComplete = 0;
	selfCheck.isAllOK = 0 ;
	
}

void bsp_SetAngleChk(uint8_t val)
{
	selfCheck.b_angle = val;
}

static void bsp_SendSelfCheck(void)
{
	selfCheck.head = 0xAAAA;
	selfCheck.frame_len = sizeof(selfCheck) & 0xFFFF;
	selfCheck.frame_len_reverse = (~selfCheck.frame_len) & 0xFFFF;
	
	selfCheck.tx_addr = 0;
	selfCheck.rx_addr = 0;
	
	selfCheck.main_sec = 2;
	selfCheck.sub_sec = 3;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&selfCheck,sizeof(selfCheck)-2);
	selfCheck.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&selfCheck,sizeof(selfCheck));
}

void bsp_SelfCheckProc(void)
{
	if(!selfCheck.isRunning)
	{
		return;
	}
	
	switch(selfCheck.action)
	{
		case 0: /*先打开全部的电机*/
		{
			bsp_SendSelfCheck();
			
			bsp_StartVacuum(VACUUM_NORMAL);
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
			
			selfCheck.delay = xTaskGetTickCount();
			++selfCheck.action;
		}break;
		
		case 1: /*稳定运行一定时间后再测量各个数据*/
		{
			if(xTaskGetTickCount() - selfCheck.delay >= 3000)
			{
				++selfCheck.action;
			}
		}break;
		
		case 2:
		{
			/*电池电压*/
			float batteryVoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
			batteryVoltage = (batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F; 
			if( batteryVoltage >= 12.0F && batteryVoltage <= 16.5F)
			{
				selfCheck.b_batteryV = 1 ;
			}
			else
			{
				selfCheck.b_batteryV = 0 ;
			}
			/*红外通信*/
			selfCheck.b_irCommunicate = bsp_IsInitAW9523B_OK();
			/*红外接收*/
			selfCheck.b_irRX_1 = SELF_CHECK_RX_CH1() == 0 ? 1 : 0;
			selfCheck.b_irRX_2 = SELF_CHECK_RX_CH2() == 0 ? 1 : 0;
			selfCheck.b_irRX_3 = SELF_CHECK_RX_CH3() == 0 ? 1 : 0;
			selfCheck.b_irRX_4 = SELF_CHECK_RX_CH4() == 0 ? 1 : 0;
			/*红外读值*/
			float vol = bsp_GetInfraRedAdcVoltage(IR7);
			if(vol >= 1000 && vol<=2000)
			{
				selfCheck.b_irRead = 1 ;
			}
			else
			{
				selfCheck.b_irRead = 0 ;
			}
			/*跳崖*/
			float cliff_arr[3] = {0};
			bsp_GetCliffSub(cliff_arr);
			if(cliff_arr[0] >= 600 && cliff_arr[0] <= 1200)
			{
				selfCheck.b_cliffL = 1 ;
			}
			else
			{
				selfCheck.b_cliffL = 0 ;
			}
			
			if(cliff_arr[1] >= 1000 && cliff_arr[1] <= 1600)
			{
				selfCheck.b_cliffM = 1 ;
			}
			else
			{
				selfCheck.b_cliffM = 0 ;
			}
			
			if(cliff_arr[2] >= 1200 && cliff_arr[2] <= 2000)
			{
				selfCheck.b_cliffR = 1 ;
			}
			else
			{
				selfCheck.b_cliffR = 0 ;
			}
			/*电流与速度*/
			float wheelL = bsp_GetFeedbackVoltage(eMotorLeft);
			float wheelR = bsp_GetFeedbackVoltage(eMotorRight);
			float roll = bsp_GetFeedbackVoltage(eRollingBrush);
			float vacuum = bsp_GetFeedbackVoltage(eVacuum);
			float sideBrush = bsp_GetFeedbackVoltage(eSideBrush);
			
			int32_t speed_l = bsp_MotorGetSpeed(MotorLeft);
			int32_t speed_r = bsp_MotorGetSpeed(MotorRight);
			
			wheelL = wheelL * 1000.0F * 1000.0F / 33.0F / 50.0F;
			wheelR = wheelR * 1000.0F * 1000.0F / 33.0F / 50.0F;
			roll = roll * 1000.0F * 1000.0F / 33.0F / 50.0F;
			vacuum = vacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
			sideBrush = sideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;
			
			
			if(wheelL >= 40 && wheelL <= 200)
			{
				selfCheck.b_wheelL_I =1 ;
			}
			else
			{
				selfCheck.b_wheelL_I =0 ;
			}
			
			
			if(wheelR >= 40 && wheelR <= 200)
			{
				selfCheck.b_wheelR_I =1 ;
			}
			else
			{
				selfCheck.b_wheelR_I =0 ;
			}
			
			if(speed_l >= 220 && speed_l <= 280)
			{
				selfCheck.b_wheelL_Speed =1 ;
			}
			else
			{
				selfCheck.b_wheelL_Speed =0 ;
			}
			
			if(speed_r >= 220 && speed_r <= 280)
			{
				selfCheck.b_wheelR_Speed =1 ;
			}
			else
			{
				selfCheck.b_wheelR_Speed =0 ;
			}
			
			if(vacuum >= 200 && vacuum <= 800)
			{
				selfCheck.b_vacuum_I =1 ;
			}
			else
			{
				selfCheck.b_vacuum_I =0 ;
			}
			
			if(roll >= 150 && roll <= 300)
			{
				selfCheck.b_rollBrush_I =1 ;
			}
			else
			{
				selfCheck.b_rollBrush_I =0 ;
			}
			
			if(sideBrush >= 50 && sideBrush <= 200)
			{
				selfCheck.b_sideBrush_I =1 ;
			}
			else
			{
				selfCheck.b_sideBrush_I =0 ;
			}
			
			/*整机电流*/
			selfCheck.b_whole_I =1 ;
			
			/*离地开关*/
			OffSiteState state = bsp_OffSiteGetState();
			
			if(state == OffSiteBoth)
			{
				selfCheck.b_offsiteL = 1 ;
				selfCheck.b_offsiteR = 1 ;
			}
			else if(state == OffSiteNone)
			{
				selfCheck.b_offsiteL = 0 ;
				selfCheck.b_offsiteR = 0 ;
			}
			else if(state == OffSiteLeft)
			{
				selfCheck.b_offsiteL = 1 ;
				selfCheck.b_offsiteR = 0 ;
			}
			else if(state == OffSiteRight)
			{
				selfCheck.b_offsiteL = 0 ;
				selfCheck.b_offsiteR = 1 ;
			}
			
			/*尘盒*/
			if(bsp_DustBoxGetState() == DustBoxInside)
			{
				selfCheck.b_hall = 1 ;
			}
			else
			{
				selfCheck.b_hall = 0 ;
			}
			
			/*测试完需要的数据之后就直接关闭电机 风机 接着测试按键和碰撞开关*/
			bsp_StopVacuum();
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.0F);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.0F);
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			
			bsp_SendSelfCheck();
			
			++selfCheck.action;
		}break;
		
		case 3:
		{
			if(SELF_CHECK_KEY_CLEAN() == 0)
			{
				selfCheck.b_touch_1 = 1 ;
				bsp_SendSelfCheck();
				++selfCheck.action;
			}
		}break;
		
		case 4:
		{
			if(SELF_CHECK_KEY_POWER() == 0)
			{
				selfCheck.b_touch_2 = 1 ;
				bsp_SendSelfCheck();
				++selfCheck.action;
			}
		}break;
		
		case 5:
		{
			if(SELF_CHECK_KEY_CHARGE() == 0)
			{
				selfCheck.b_touch_3 = 1 ;
				bsp_SendSelfCheck();
				++selfCheck.action;
			}
		}break;
		
		case 6:
		{
			if(bsp_CollisionScan() == CollisionLeft)
			{
				selfCheck.b_collision_1 = 1 ;
				bsp_SendSelfCheck();
				++selfCheck.action;
			}
		}break;
		
		case 7:
		{
			if(bsp_CollisionScan() == CollisionRight)
			{
				selfCheck.b_collision_2 = 1 ;
				bsp_SendSelfCheck();
				++selfCheck.action;
			}
		}break;
		
		case 8:
		{
			if(
					selfCheck.b_batteryV          &&
					selfCheck.b_irCommunicate     &&
					selfCheck.b_irRX_1            &&
					selfCheck.b_irRX_2            &&
					selfCheck.b_irRX_3            &&
					selfCheck.b_irRX_4            &&
					selfCheck.b_irRead            &&
					selfCheck.b_cliffL            &&
					selfCheck.b_cliffM            &&
					selfCheck.b_cliffR            &&
					selfCheck.b_wheelL_I          &&
					selfCheck.b_wheelR_I          &&
					selfCheck.b_wheelL_Speed      &&
					selfCheck.b_wheelR_Speed      &&
					selfCheck.b_offsiteL          &&
					selfCheck.b_offsiteR          &&
					selfCheck.b_vacuum_I          &&
					selfCheck.b_rollBrush_I       &&
					selfCheck.b_sideBrush_I       &&
					selfCheck.b_whole_I           &&
					selfCheck.b_angle             &&
					selfCheck.b_touch_1           &&
					selfCheck.b_touch_2           &&
					selfCheck.b_touch_3           &&
					selfCheck.b_collision_1       &&
					selfCheck.b_collision_2       &&
					selfCheck.b_hall
				)
			{
				selfCheck.isAllOK = 1 ;
			}
			
			selfCheck.isChkAllComplete = 1 ;
			
			selfCheck.delay = xTaskGetTickCount();
			++selfCheck.action;
		}break;
		
		case 9:
		{
			if(xTaskGetTickCount() - selfCheck.delay >= 100)
			{
				bsp_SendSelfCheck();
				
				

				
				selfCheck.delay = xTaskGetTickCount();
				++selfCheck.action;
			}
		}break;
		
		case 10:
		{
			if(xTaskGetTickCount() - selfCheck.delay >= 1000 && selfCheck.isAllOK)
			{
				/*进入休眠模式*/
				bsp_SperkerPlay(Song31);
				vTaskDelay(10);	
				while(bsp_SpeakerIsBusy()){}
				
				bsp_ClearKey();
				bsp_EnterStopMODE();
					
				bsp_StopSelfCheck();
			}
		}break;
	}
}



