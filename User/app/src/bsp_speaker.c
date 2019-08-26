#include "bsp.h"

/* 扬声器 */
#define RCC_ALL_SPEAKER 	(RCC_APB2Periph_GPIOD)

#define GPIO_PORT_SPEAKER_DI      GPIOD
#define GPIO_PIN_SPEAKER_DI	      GPIO_Pin_2
                                  
#define GPIO_PORT_SPEAKER_RST     GPIOD
#define GPIO_PIN_SPEAKER_RST	  GPIO_Pin_1

#define GPIO_PORT_SPEAKER_BUSY    GPIOD
#define GPIO_PIN_SPEAKER_BUSY	  GPIO_Pin_0





/*
*********************************************************************************************************
*	函 数 名: bsp_InitSpeaker
*	功能说明: 初始化扬声器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitSpeaker(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_SPEAKER, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	/*DI引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_DI;
	GPIO_Init(GPIO_PORT_SPEAKER_DI, &GPIO_InitStructure);

	/*RST引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_RST;
	GPIO_Init(GPIO_PORT_SPEAKER_RST, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	/*BUSY引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_BUSY;
	GPIO_Init(GPIO_PORT_SPEAKER_BUSY, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SperkerPlay
*	功能说明: 播放指定声音，在播放指定声音之前，应该首先检测模块是否处于忙状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SperkerPlay(SongSN sn)
{
	uint16_t i = 0 ;
	
	/*复位*/
	GPIO_SetBits(GPIO_PORT_SPEAKER_RST,GPIO_PIN_SPEAKER_RST);
	bsp_DelayUS(100);
	GPIO_ResetBits(GPIO_PORT_SPEAKER_RST,GPIO_PIN_SPEAKER_RST);
	bsp_DelayUS(100);
	
	/*根据曲目信息，发送曲目*/
	for(i=0;i<sn;i++)
	{
		GPIO_SetBits(GPIO_PORT_SPEAKER_DI,GPIO_PIN_SPEAKER_DI);
		bsp_DelayUS(100);
		GPIO_ResetBits(GPIO_PORT_SPEAKER_DI,GPIO_PIN_SPEAKER_DI);
		bsp_DelayUS(100);
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SpeakerIsBusy
*	功能说明: 检测扬声器是否忙状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_SpeakerIsBusy(void)
{
	bool ret = 0 ;
	
	if(GPIO_ReadInputDataBit(GPIO_PORT_SPEAKER_BUSY,GPIO_PIN_SPEAKER_BUSY))
	{
		ret = true;
	}
	else
	{
		ret = false;
	}
	
	return ret;
}


