#include "bsp.h"

#define RANGE			100         /*����������������Χ*/
#define RangeSub(x)		((x)-RANGE) /*����������������Χ ��*/
#define RangeAdd(x)		((x)+RANGE) /*����������������Χ ��*/
#define CYCLE			80          /*���ȷ�ϼ�����Ӧ��ȵ����壬*/

typedef struct
{
	volatile bool is500us;
	volatile bool is1000us;
	volatile bool is1500us;
	volatile uint32_t remoteStatistics[3]; //����ͳ�Ƽ�������
}Remote;

typedef enum
{
	Pulse500US  = 0 ,
	Pulse1000US = 1 ,
	Pulse1500US = 2 
}PulseWidth;

static volatile Remote remote[4];
static volatile uint32_t remoteStateTimer[4][3]; //�������壬4�����չ�


typedef struct
{
	uint8_t isRunning;
	uint32_t action ;
	uint32_t delay;
	float angle;
	bool isNeedBack;
	bool isRight;
}SearchCharging;

static ChargingPile chargingPile;
static SearchCharging searchCharging;
static void bsp_InitTIM3Cap(u16 arr,u16 psc);
static void bsp_InitIO(void);

void bsp_InitChargingPile(void)
{
    bsp_InitTIM3Cap(0xFFFF,72-1); //1MHz������
	
	bsp_InitIO();
	
}



void bsp_StartSearchChargingPile(void)
{
	searchCharging.action = 0 ;
	searchCharging.isRunning = 1 ;
}

void bsp_StopSearchChargingPile(void)
{
	searchCharging.isRunning = 0 ;
	searchCharging.action = 0 ;
}

void bsp_SearchChargingPileAct(void)
{
	if(!searchCharging.isRunning)
	{
		return ;
	}
	
	/*���*/
	if(bsp_GetChargeFeedback() == true)
	{
		DEBUG("is charging...\r\n");
		bsp_SetMotorSpeed(MotorLeft,0);
		bsp_SetMotorSpeed(MotorRight,0);
		bsp_StopSearchChargingPile();
		return ;
	}
	
	switch(searchCharging.action)
	{
		case 0: //��һ����ֱ��
		{
			bsp_SetMotorSpeed(MotorLeft,5);
			bsp_SetMotorSpeed(MotorRight,5);
			
			searchCharging.action++;
		}break;
		
		case 1: //ֱ��1���ܹ���⵽��������
		{
			if(remote[CapCH1].is500us && remote[CapCH1].is1000us && remote[CapCH1].is1500us)
			{
				DEBUG("1 detect 3 pulse\r\n");
				bsp_SetMotorSpeed(MotorLeft,0);
				bsp_SetMotorSpeed(MotorRight,0);

				searchCharging.isRight = true ; //���ұ�����
				searchCharging.action++;
			}
			if(remote[CapCH2].is500us && remote[CapCH2].is1000us && remote[CapCH2].is1500us)
			{
				DEBUG("2 detect 3 pulse\r\n");
				bsp_SetMotorSpeed(MotorLeft,0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				searchCharging.isRight = false ; //���������
				searchCharging.action++;
			}
			else if((remote[CapCH4].is500us && remote[CapCH4].is1000us)  || (remote[CapCH3].is500us && remote[CapCH3].is1000us))
			{
				searchCharging.action = 5 ;
			}
		}break;
		

		case 2: //�����Ҳ���
		{
			if(searchCharging.isRight) //�ұ�����
			{
				bsp_SetMotorSpeed(MotorLeft,5);
				searchCharging.action++;
			}
			else //�������
			{
				bsp_SetMotorSpeed(MotorRight,5);
				searchCharging.action++;
			}
			
		}break;
		
		
		case 3: //4��ͬʱ�յ�
		{
			if(searchCharging.isRight) //�ұ�����
			{
				if(remote[CapCH4].is500us && remote[CapCH4].is1000us && (remote[CapCH3].is500us || remote[CapCH3].is1000us))
				{
					DEBUG("4 detect 3 pulse\r\n");
					//bsp_MotorBrake(MotorLeft);
					//bsp_MotorBrake(MotorRight);
					searchCharging.action++;
				}
			}
			else //�������
			{
				if(remote[CapCH3].is500us && remote[CapCH3].is1000us && (remote[CapCH4].is500us || remote[CapCH4].is1000us))
				{
					DEBUG("3 detect 3 pulse\r\n");
					//bsp_MotorBrake(MotorLeft);
					//bsp_MotorBrake(MotorRight);
					searchCharging.action++;
				}
			}
		}break;
		
		case 4: //��ת
		{
			if(searchCharging.isRight) //�ұ�����
			{
				//bsp_SetMotorTargetSpeed(MotorLeft, 180);
				//bsp_SetMotorTargetSpeed(MotorRight,140);
				searchCharging.action++;	
			}
			else
			{
				//bsp_SetMotorTargetSpeed(MotorLeft, 140);
				//bsp_SetMotorTargetSpeed(MotorRight,180);
				searchCharging.action++;	
			}				
		}break;
		
		
		case 5: //ǰ��һ�����ղ�����
		{
			Collision collision = bsp_CollisionScan();
			if(collision != CollisionNone)
			{
				//bsp_MotorBrake(MotorLeft);
				//bsp_MotorBrake(MotorRight);
				//bsp_SetMotorTargetSpeed(MotorLeft, -180);
				//bsp_SetMotorTargetSpeed(MotorRight,-180);
				
				searchCharging.isNeedBack = true;
				searchCharging.delay = xTaskGetTickCount();
			}
			else if(remote[CapCH3].is500us && remote[CapCH4].is1000us) //ͬʱ��ֱ��
			{
				DEBUG("both\r\n");
				//bsp_SetMotorTargetSpeed(MotorLeft, 140);
				//bsp_SetMotorTargetSpeed(MotorRight,140);
			}
			else if(remote[CapCH1].is500us || remote[CapCH1].is1000us)
			{
				DEBUG("adjust\r\n");
				//bsp_SetMotorTargetSpeed(MotorLeft, 180);
				//bsp_SetMotorTargetSpeed(MotorRight,120);
			}
			else if(!remote[CapCH4].is1000us) //4�ղ���1000
			{
				DEBUG("4 miss 1000\r\n");
				//bsp_SetMotorTargetSpeed(MotorLeft, 140);
				//bsp_SetMotorTargetSpeed(MotorRight,180);
			}
			else if(!remote[CapCH3].is500us) //3�ղ���500
			{
				DEBUG("3 miss 500\r\n");
				//bsp_SetMotorTargetSpeed(MotorLeft, 180);
				//bsp_SetMotorTargetSpeed(MotorRight,140);
			}
			
			
			searchCharging.action++;
		}break;
		
		case 6:
		{
			if(searchCharging.isNeedBack)
			{
				if(xTaskGetTickCount() - searchCharging.delay >= 5000)
				{
					searchCharging.isNeedBack = false;
				}
			}
			else
			{
				searchCharging.action = 5;
			}
		}break;
		
		
		
	}
}






/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearRemoteTimerCnt
*	����˵��: ָ�����չܣ��������ͣ������������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ClearRemoteTimerCnt(CapCH capCH , PulseWidth pulseWidth)
{
	remoteStateTimer[capCH][pulseWidth] = 0 ;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearRemoteTimerCnt
*	����˵��: ָ�����չܣ��������ͣ���������1��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_RemoteTimerCntAdd(CapCH capCH , PulseWidth pulseWidth)
{
	++remoteStateTimer[capCH][pulseWidth];
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearRemotePulseState
*	����˵��: ���ָ���ܣ�ָ�����������䷶Χ״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ClearRemotePulseState(CapCH capCH , PulseWidth pulseWidth)
{
	switch(pulseWidth)
	{
		case Pulse500US:
		{
			remote[capCH].is500us = false ;
		}break;
		case Pulse1000US:
		{
			remote[capCH].is1000us = false ;
		}break;
		case Pulse1500US:
		{
			remote[capCH].is1500us = false ;
		}break;
	}

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetRemotePulseState
*	����˵��: �趨ָ���ܣ�ָ�����������䷶Χ״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetRemotePulseState(CapCH capCH , PulseWidth pulseWidth)
{
	switch(pulseWidth)
	{
		case Pulse500US:
		{
			remote[capCH].is500us = true ;
		}break;
		case Pulse1000US:
		{
			remote[capCH].is1000us = true ;
		}break;
		case Pulse1500US:
		{
			remote[capCH].is1500us = true ;
		}break;
	}

}



void bsp_PulseTimerPer1MS(void)
{
	CapCH ch = CapCH1 ;
	PulseWidth pulse = Pulse500US ;
	
	for(ch=CapCH1;ch<=CapCH4;ch++)
	{
		for(pulse=Pulse500US;pulse<=Pulse1500US;pulse++)
		{
			if(remoteStateTimer[ch][pulse] <= CYCLE)
			{
				bsp_RemoteTimerCntAdd(ch,pulse); /*�������ֵΪ0����ÿMS��1��ֱ��һ��ѭ������*/
			}
			else
			{
				if(pulse == Pulse500US && remote[ch].is500us==true)
				{
					remote[ch].is500us = false;
				}
				else if(pulse == Pulse1000US && remote[ch].is1000us==true)
				{
					remote[ch].is1000us = false;
				}
				else if(pulse == Pulse1500US && remote[ch].is1500us==true)
				{
					remote[ch].is1500us = false;
				}
			}
		}
	}
	
}





uint32_t bsp_GetCapCnt(CapCH capCH)
{
	uint32_t temp = 0; 
	CapCH ch = capCH;
	
	if(chargingPile.capState[ch]&0X80)//�ɹ�������һ��������
	{
		temp=chargingPile.capState[ch]&0X3F;
		temp*=65536;//���ʱ���ܺ�
		temp+=chargingPile.capValue[ch];//�õ��ܵ�ʱ��
		temp-=chargingPile.capStart[ch];//�õ��ܵ�ʱ��
		
		
		//printf("LOW:%d us\r\n",temp);   //��ӡ�ܵ�ʱ��

		if(temp >= RangeSub(500) && temp <=  RangeAdd(500))
		{
			remote[ch].remoteStatistics[1] = 0 ;
			remote[ch].remoteStatistics[2] = 0 ;
			
			if(++remote[ch].remoteStatistics[0] >= 3)
			{
				remote[ch].is500us = true;
				bsp_ClearRemoteTimerCnt(ch,Pulse500US);//�������
			}
		}
		else if(temp >= RangeSub(1000) && temp <=  RangeAdd(1000))
		{
			remote[ch].remoteStatistics[0] = 0 ;
			remote[ch].remoteStatistics[2] = 0 ;
			
			if(++remote[ch].remoteStatistics[1] >= 3)
			{
				remote[ch].is1000us = true;
				bsp_ClearRemoteTimerCnt(ch,Pulse1000US);//�������
			}
		}
		else if(temp >= RangeSub(1500) && temp <=  RangeAdd(1500))
		{
			remote[ch].remoteStatistics[0] = 0 ;
			remote[ch].remoteStatistics[1] = 0 ;
			
			if(++remote[ch].remoteStatistics[2] >= 3)
			{
				remote[ch].is1500us = true;
			bsp_ClearRemoteTimerCnt(ch,Pulse1500US);//�������
			}
			
		}
		
		chargingPile.capState[ch]=0;
		chargingPile.capValue[ch]=0;
	}
	
	return temp;
}


void bsp_PrintRemoteState(CapCH capCH)
{
	printf("*******CH(1000,1500,500)******\r\n");
	printf("CH1:%d %d %d\r\n",remote[CapCH1].is1000us,remote[CapCH1].is1500us,remote[CapCH1].is500us);
	printf("CH2:%d %d %d\r\n",remote[CapCH2].is1000us,remote[CapCH2].is1500us,remote[CapCH2].is500us);
	printf("CH3:%d %d %d\r\n",remote[CapCH3].is1000us,remote[CapCH3].is1500us,remote[CapCH3].is500us);
	printf("CH4:%d %d %d\r\n",remote[CapCH4].is1000us,remote[CapCH4].is1500us,remote[CapCH4].is500us);
}



static void bsp_InitTIM3Cap(u16 arr,u16 psc)
{	 
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period 		= arr; 				    //�趨�������Զ���װֵ ���10ms���  
    TIM_TimeBaseStructure.TIM_Prescaler     = psc; 				    //Ԥ��Ƶ��,1M�ļ���Ƶ��,1us��1.	   	
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
    
    
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 	
    TIM_ICInitStructure.TIM_ICFilter    = 0x03;					
    
    TIM_ICInitStructure.TIM_Channel 	= TIM_Channel_1;  		
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel 	= TIM_Channel_2;  		
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel 	= TIM_Channel_3;  		
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel 	= TIM_Channel_4;  		
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    
    NVIC_InitStructure.NVIC_IRQChannel 			  = TIM3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		//�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd 		  = ENABLE; 	//IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   	
    
    TIM_Cmd(TIM3,ENABLE ); 	
    
    
    TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
}





void TIM3_IRQHandler(void)
{
	CapCH ch = CapCH1;
	
	
	/************************  CH1  *********************************/
	ch = CapCH1;
	if((chargingPile.capState[ch]&0X80) == 0)
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
        {	    
            if(chargingPile.capState[ch]&0X40)
            {
                if((chargingPile.capState[ch]&0X3F)==0X3F)
                {
                    chargingPile.capState[ch]|=0X80;
                    chargingPile.capValue[ch]=0XFFFF;
                }else chargingPile.capState[ch]++;
            }	 
        }
        if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
        {	
            if(chargingPile.capState[ch]&0X40)			
            {	  			
                chargingPile.capState[ch]|=0X80;
                chargingPile.capValue[ch]=TIM_GetCapture1(TIM3);
                TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);
            }else 
            {
                chargingPile.capState[ch]=0;	
                chargingPile.capValue[ch]=0;
				chargingPile.capStart[ch]=TIM_GetCounter(TIM3);
                chargingPile.capState[ch]|=0X40;
                TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);
            }		    
        }			 
	}
	
	
	/************************  CH2  *********************************/
	ch = CapCH2;
	if((chargingPile.capState[ch]&0X80) == 0)
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
        {	    
            if(chargingPile.capState[ch]&0X40)
            {
                if((chargingPile.capState[ch]&0X3F)==0X3F)
                {
                    chargingPile.capState[ch]|=0X80;
                    chargingPile.capValue[ch]=0XFFFF;
                }else chargingPile.capState[ch]++;
            }	 
        }
        if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
        {	
            if(chargingPile.capState[ch]&0X40)			
            {	  			
                chargingPile.capState[ch]|=0X80;
                chargingPile.capValue[ch]=TIM_GetCapture2(TIM3);
                TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);
            }else 
            {
                chargingPile.capState[ch]=0;	
                chargingPile.capValue[ch]=0;
				chargingPile.capStart[ch]=TIM_GetCounter(TIM3);
                chargingPile.capState[ch]|=0X40;
                TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising);
            }		    
        }			 
	}
	
	
	/************************  CH3  *********************************/
	ch = CapCH3;
	if((chargingPile.capState[ch]&0X80) == 0)
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
        {	    
            if(chargingPile.capState[ch]&0X40)
            {
                if((chargingPile.capState[ch]&0X3F)==0X3F)
                {
                    chargingPile.capState[ch]|=0X80;
                    chargingPile.capValue[ch]=0XFFFF;
                }else chargingPile.capState[ch]++;
            }	 
        }
        if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
        {	
            if(chargingPile.capState[ch]&0X40)			
            {	  			
                chargingPile.capState[ch]|=0X80;
                chargingPile.capValue[ch]=TIM_GetCapture3(TIM3);
                TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);
            }else 
            {
                chargingPile.capState[ch]=0;	
                chargingPile.capValue[ch]=0;
				chargingPile.capStart[ch]=TIM_GetCounter(TIM3);
                chargingPile.capState[ch]|=0X40;
                TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising);
            }		    
        }			 
	}
	
	
	/************************  CH4  *********************************/
	ch = CapCH4;
	if((chargingPile.capState[ch]&0X80) == 0)
	{
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
        {	    
            if(chargingPile.capState[ch]&0X40)
            {
                if((chargingPile.capState[ch]&0X3F)==0X3F)
                {
                    chargingPile.capState[ch]|=0X80;
                    chargingPile.capValue[ch]=0XFFFF;
                }else chargingPile.capState[ch]++;
            }	 
        }
        if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
        {	
            if(chargingPile.capState[ch]&0X40)			
            {	  			
                chargingPile.capState[ch]|=0X80;
                chargingPile.capValue[ch]=TIM_GetCapture4(TIM3);
                TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);
            }else 
            {
                chargingPile.capState[ch]=0;	
                chargingPile.capValue[ch]=0;
				chargingPile.capStart[ch]=TIM_GetCounter(TIM3);
                chargingPile.capState[ch]|=0X40;
                TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);
            }		    
        }			 
	}
	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //����жϱ�־λ
    
}


static void bsp_InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
 
}



/*
*********************************************************************************************************
*	�� �� ��: AdcPro
*	����˵��: ADC��������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_GetChargeFeedback(void)
{
	if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_7))
	{
		return true ;
	}
	else
	{
		return false ;
	}
}


