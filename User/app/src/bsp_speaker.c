#include "bsp.h"

/* ������ ,ԭ��ͼ��RST��DI�����Ƿ���*/
#define RCC_ALL_SPEAKER 	(RCC_APB2Periph_GPIOA)

#define GPIO_PORT_SPEAKER_DI      GPIOA
#define GPIO_PIN_SPEAKER_DI	      GPIO_Pin_10
                                  
#define GPIO_PORT_SPEAKER_RST     GPIOA
#define GPIO_PIN_SPEAKER_RST	  GPIO_Pin_9

#define GPIO_PORT_SPEAKER_BUSY    GPIOA
#define GPIO_PIN_SPEAKER_BUSY	  GPIO_Pin_8



/*
*********************************************************************************************************
*	�� �� ��: bsp_InitSpeaker
*	����˵��: ��ʼ��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitSpeaker(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_SPEAKER, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	
	/*DI����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_DI;
	GPIO_Init(GPIO_PORT_SPEAKER_DI, &GPIO_InitStructure);

	/*RST����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_RST;
	GPIO_Init(GPIO_PORT_SPEAKER_RST, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	/*BUSY����*/
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_SPEAKER_BUSY;
	GPIO_Init(GPIO_PORT_SPEAKER_BUSY, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SperkerPlay
*	����˵��: ����ָ���������ڲ���ָ������֮ǰ��Ӧ�����ȼ��ģ���Ƿ���æ״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SperkerPlay(SongSN sn)
{
	uint16_t i = 0 ;
	
	/*��λ*/
	GPIO_SetBits(GPIO_PORT_SPEAKER_RST,GPIO_PIN_SPEAKER_RST);
	bsp_DelayUS(100);
	GPIO_ResetBits(GPIO_PORT_SPEAKER_RST,GPIO_PIN_SPEAKER_RST);
	bsp_DelayUS(100);
	
	/*������Ŀ��Ϣ��������Ŀ*/
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
*	�� �� ��: bsp_SpeakerIsBusy
*	����˵��: ����������Ƿ�æ״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_SpeakerIsBusy(void)
{
	bool ret = 0 ;
	
	vTaskDelay(200);
	
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


