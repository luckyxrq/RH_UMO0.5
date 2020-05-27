#include "bsp.h"
#include <math.h>


static int int_abs(int x){
    if (x<0){
        x= -x;
    }
    return x;
}
static FunctionTest functiontest;
static	uint8_t FunctionTestStep = 0;
static	int last_X,last_Y =  0;
static	int yaw = 0;



void bsp_StartFunctionTest(void)
{
	functiontest.action = 0 ;
	functiontest.delay = 0 ;
	functiontest.isRunning = true;
}

void bsp_StopFunctionTest(void)
{
	functiontest.isRunning = false;
	functiontest.action = 0 ;
	functiontest.delay = 0 ;
	FunctionTestStep = 0;
}

void bsp_FunctionTestUpdate(void)
{
	if(!functiontest.isRunning)
	{
		return ;
	}
	
	switch(functiontest.action)
	{
		case 0:
		{
			
			switch (FunctionTestStep)
			{

				case FTS_ReadyGo:
					bsp_StartEdgewiseRun();
					FunctionTestStep = FTS_RightEdgewiseRun;
					//goto FTS_RightEdgewiseRun
					//start right edgewiserun
					break;
				case FTS_RightEdgewiseRun:
					bsp_EdgewiseRun();
					if((bsp_GetInfraRedAdcVoltage(IR7)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)
					{
						if((bsp_GetInfraRedAdcVoltage(IR7)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)
						{
							if((bsp_GetInfraRedAdcVoltage(IR7)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)
							{
								bsp_StopEdgewiseRun();
								FunctionTestStep = FTS_ErLangStartTurnAround;
								bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
								bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
							}
						}
					}
					//right edgewiserun    
					//if £¨erlang == ture £© stop edgewiserun ,goto FTS_ErLangStartTurnAround
					break;
				case FTS_ErLangStartTurnAround:

					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(50));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(200));	
					if( Rad2Deg(bsp_GetCurrentOrientation()) > 120 )
					{
						FunctionTestStep = FTS_ErlangGostraight;
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
						last_X = bsp_GetCurrentPosX();
						last_Y = bsp_GetCurrentPosY();
					}

					//turn around cclock turn radius 20CM   
					//if £¨yaw > 130£© stop turn around ,goto FTS_TurnAroundErlangStop
					break;
				case FTS_ErlangGostraight:
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(200));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(200));	
					if(int_abs(last_X - bsp_GetCurrentPosX()) > 200 )
					{
						FunctionTestStep = FTS_ErLangStopTurnAround;
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					}
					//gostraight   if £¨¡÷X > 0.3m£©,goto FTS_ErLangStopTurnAround
					break;
				case FTS_ErLangStopTurnAround:
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(200));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(50));	
					if( Rad2Deg(bsp_GetCurrentOrientation()) <45 )
					{
						FunctionTestStep = FTS_CliffGosraight;
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					}
					//turn around clock turn radius 20CM   
					//if £¨yaw < 45£© stop turn around ,goto FTS_CliffStart
					break;
				case FTS_CliffGosraight:  
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(200));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(200));	
					bsp_GetCliffSensorData();
					if(cliff_valueB.cliffValue0 ==1)
					{
						FunctionTestStep = FTS_CliffBackward;
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
						last_X = bsp_GetCurrentPosX();
						last_Y = bsp_GetCurrentPosY();
					}
					//go straight if£¨cliff == ture£©£¬goto CliffBackward
					break;
				case FTS_CliffBackward:  
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-200));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));	
					if(int_abs(last_X - bsp_GetCurrentPosX()) > 200 )
					{
						FunctionTestStep = FTS_CliffStartTurnAround;
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					}
					//Backward  if£¨¡÷X > 0.2m£©£¬goto FTS_CliffStartTurnAround
					break;
				case FTS_CliffStartTurnAround:
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(-200));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+200));	
				
					yaw  = Rad2Deg(bsp_GetCurrentOrientation());
					if( yaw > -165  && yaw < 0  )
					{
						 
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
						bsp_StartEdgewiseRun();
						FunctionTestStep = FTS_LeftEdgewiseRun;
						
					}
					////turn around cclock turn radius 0CM   
					//if £¨yaw > -165  && yaw < 0£© stop turn around,start left edgewiserun ,goto FTS_LeftEdgewiseRun
					break;
				case FTS_LeftEdgewiseRun: 
					bsp_EdgewiseRun();	
					if(bsp_GetCurrentPosX() <1000)
					{
						bsp_StopEdgewiseRun();
						FunctionTestStep = FTS_SearchChargePile;
						bsp_StartSearchChargePile();
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					}
					//left edgewiserun  
					//if (¡÷X > 1m) ,stop left edgewiserun£¬StartSearchChargePile,goto FTS_SearchChargePile
					break;
				case FTS_SearchChargePile: 
					bsp_SearchChargePile();
					if(bsp_IsTouchChargePile())
					{
						bsp_StopSearchChargePile();
						bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
						FunctionTestStep = FTS_AllComplete;
					}
					//SearchChargePile();
					break;
				case FTS_AllComplete:
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					break;
				default:
					break;
				
				
			}
			
		}break;
		default: break;
	}
}