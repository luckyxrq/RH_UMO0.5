#include "bsp.h"

typedef struct
{
	uint8_t isRunning;
	uint32_t action ;
	
}SearchCharging;



static ChargingPile chargingPile;
static SearchCharging searchCharging;

static void bsp_InitTIM3Cap(u16 arr,u16 psc);


void bsp_InitChargingPile(void)
{
    bsp_InitTIM3Cap(0xFFFF,72-1); //1MHz采样率，重载值1000，1000US中断一次
	
	bsp_StartSearchChargingPile();
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
	
	switch(searchCharging.action)
	{
		case 0:
		{
			
		}break;
	}
}

uint32_t bsp_GetCapCnt(CapCH capCH)
{
	uint32_t temp = 0; 
	CapCH ch = capCH;
	
	if(chargingPile.capState[ch]&0X80)//成功捕获到了一次上升沿
	{
		temp=chargingPile.capState[ch]&0X3F;
		temp*=65536;//溢出时间总和
		temp+=chargingPile.capValue[ch];//得到总的时间
		temp-=chargingPile.capStart[ch];//得到总的时间
		printf("LOW:%d us\r\n",temp);   //打印总的时间
		chargingPile.capState[ch]=0;
		chargingPile.capValue[ch]=0;
	}
	
	return temp;
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
    
    TIM_TimeBaseStructure.TIM_Period 		= arr; 				    //设定计数器自动重装值 最大10ms溢出  
    TIM_TimeBaseStructure.TIM_Prescaler     = psc; 				    //预分频器,1M的计数频率,1us加1.	   	
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  	//TIM向上计数模式
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		//从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd 		  = ENABLE; 	//IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   	
    
    TIM_Cmd(TIM3,ENABLE ); 	
    
    
    TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
}






typedef struct
{
	bool is500us;
	bool is1000us;
	bool is1500us;
}Remote;

static Remote remote[4];






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
	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
    
}


