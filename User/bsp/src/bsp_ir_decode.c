/*
*********************************************************************************************************
*
*	ģ������ : ����ң�ؽ���������ģ��
*	�ļ����� : bsp_ir_decode.c
*	��    �� : V1.0
*	˵    �� : ����ң�ؽ��յĺ����ź�����CPU�� PB0/TIM3_CH3.  ����������ʹ��TIM3_CH3ͨ�������벶������
*				Э�����롣 
*				TIM3��֧��˫���ش���, �����Ҫ���ж����л����ԡ�
*				��TIM6��TIM7֮��Ķ�ʱ����ֻ�ܲ��������ػ����½��ز�׽�����ܲ���˫���ز�׽.
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2015-07-11   armfly  ��ʽ����
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"


#define IR_UPDATE_T             1000 /* �����ʱ�����º�����䷶Χ״̬��ʵ��һ��ʱ��Ϊ73.75MS����������*/

/* ����GPIO�˿� */
#define RCC_IRD		RCC_APB2Periph_GPIOC
#define PORT_IRD	GPIOC
#define PIN_IRD		(GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)


static IRD_T g_tIR;

static void bsp_IR_SoftTimerInit(void);


/*
*********************************************************************************************************
*	�� �� ��: bsp_IR_GetRev
*	����˵��: ��ȡÿ��������չܵ�ÿ��ͨ���Ľ�����ֵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IR_GetRev(IR_CH ch , IRSite site)
{
	return g_tIR.isRev[ch][site];
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PrintIR_Rev
*	����˵��: ��ӡ�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PrintIR_Rev(void)
{
	DEBUG("     CH1  %d  %d  %d      ",
	g_tIR.isRev[IR_CH1][IR_TX_SITE_LEFT],
	g_tIR.isRev[IR_CH1][IR_TX_SITE_CENTER],
	g_tIR.isRev[IR_CH1][IR_TX_SITE_RIGHT]);
	
	DEBUG("CH2  %d  %d  %d      ",
	g_tIR.isRev[IR_CH2][IR_TX_SITE_LEFT],
	g_tIR.isRev[IR_CH2][IR_TX_SITE_CENTER],
	g_tIR.isRev[IR_CH2][IR_TX_SITE_RIGHT]);
	
	DEBUG("CH3  %d  %d  %d      ",
	g_tIR.isRev[IR_CH3][IR_TX_SITE_LEFT],
	g_tIR.isRev[IR_CH3][IR_TX_SITE_CENTER],
	g_tIR.isRev[IR_CH3][IR_TX_SITE_RIGHT]);
	
	DEBUG("CH4  %d  %d  %d    ",
	g_tIR.isRev[IR_CH4][IR_TX_SITE_LEFT],
	g_tIR.isRev[IR_CH4][IR_TX_SITE_CENTER],
	g_tIR.isRev[IR_CH4][IR_TX_SITE_RIGHT]);
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_IR_SoftTick
*	����˵��: ÿ���������ȸ�״ֵ̬���Ƿ��ڷ��䷶Χ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_IR_SoftTick(IR_CH ch , IRSite site)
{
	/*����յ������ˣ��������ʱ��ֵһֱ��0*/
	if(g_tIR.isRev[ch][site])
	{
		if(++g_tIR.softTimer[ch][site] >= IR_UPDATE_T)
		{
			g_tIR.softTimer[ch][site] = 0 ;
			g_tIR.isRev[ch][site] = false;
		}
	}
	else
	{
		g_tIR.softTimer[ch][site] = 0 ;
	}
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_IR_SoftTimerTickPerMS
*	����˵��: ÿ���������ȸ�״ֵ̬���Ƿ��ڷ��䷶Χ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_IR_SoftTimerTickPerMS(void)
{
	bsp_IR_SoftTick(IR_CH1,IR_TX_SITE_LEFT);
	bsp_IR_SoftTick(IR_CH1,IR_TX_SITE_CENTER);
	bsp_IR_SoftTick(IR_CH1,IR_TX_SITE_RIGHT);
	
	bsp_IR_SoftTick(IR_CH2,IR_TX_SITE_LEFT);
	bsp_IR_SoftTick(IR_CH2,IR_TX_SITE_CENTER);
	bsp_IR_SoftTick(IR_CH2,IR_TX_SITE_RIGHT);
	
	bsp_IR_SoftTick(IR_CH3,IR_TX_SITE_LEFT);
	bsp_IR_SoftTick(IR_CH3,IR_TX_SITE_CENTER);
	bsp_IR_SoftTick(IR_CH3,IR_TX_SITE_RIGHT);
	
	bsp_IR_SoftTick(IR_CH4,IR_TX_SITE_LEFT);
	bsp_IR_SoftTick(IR_CH4,IR_TX_SITE_CENTER);
	bsp_IR_SoftTick(IR_CH4,IR_TX_SITE_RIGHT);
}



/*
*********************************************************************************************************
*	�� �� ��: IRD_StartWork
*	����˵��: ����TIM����ʼ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_IRD_StartWork(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue;
	
	/* ʱ�ӣ���ӳ�� */
	RCC_APB2PeriphClockCmd(RCC_IRD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/* ����Ϊ�������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* ����ģʽ */
	GPIO_InitStructure.GPIO_Pin = PIN_IRD;
	GPIO_Init(PORT_IRD, &GPIO_InitStructure);	
	
	/* ��ʱ��3�жϷ��� */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ���÷�ƵΪ, ���������ֵ�ĵ�λ������ 10us, ��������Ƚϡ� */
	PrescalerValue = 72000000/100000 - 1;
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/*���벶���������*/
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/*����TIM3��TIM_ICPolarity_BothEdge�������ã��Լ��л����½���*/ 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* ÿ�����䶼����1�β����¼� */
	TIM_ICInitStructure.TIM_ICFilter = 0x0;	
	
	/*ÿ��ͨ��*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	/*��������жϺ����벶���ж�*/
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	/* ����ж�ʹ�ܣ����ڳ�ʱͬ������ */
	
	/*��ʼ״̬*/
	g_tIR.LastCapture[IR_CH1] = 0;	
	g_tIR.Status[IR_CH1] = 0;
	g_tIR.WaitFallEdge[IR_CH1] = 1;	/* 0 ��ʾ�ȴ������أ�1��ʾ�ȴ��½��أ������л����벶���� */
	
	g_tIR.LastCapture[IR_CH2] = 0;	
	g_tIR.Status[IR_CH2] = 0;
	g_tIR.WaitFallEdge[IR_CH2] = 1;	/* 0 ��ʾ�ȴ������أ�1��ʾ�ȴ��½��أ������л����벶���� */
	
	g_tIR.LastCapture[IR_CH3] = 0;	
	g_tIR.Status[IR_CH3] = 0;
	g_tIR.WaitFallEdge[IR_CH3] = 1;	/* 0 ��ʾ�ȴ������أ�1��ʾ�ȴ��½��أ������л����벶���� */
	
	g_tIR.LastCapture[IR_CH4] = 0;	
	g_tIR.Status[IR_CH4] = 0;
	g_tIR.WaitFallEdge[IR_CH4] = 1;	/* 0 ��ʾ�ȴ������أ�1��ʾ�ȴ��½��أ������л����벶���� */
	
	bsp_IR_SoftTimerInit();
	
	/* ʹ�ܶ�ʱ�� */
	TIM_Cmd(TIM3, ENABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: IRD_StopWork
*	����˵��: ֹͣ�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_IRD_StopWork(void)
{
	TIM_Cmd(TIM3, DISABLE);
	
	TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);	
	TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);	
	TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);		
}

/*
*********************************************************************************************************
*	�� �� ��: IRD_DecodeNec
*	����˵��: ����NEC�����ʽʵʱ����
*	��    ��: _width �����ȣ���λ 10us
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_IRD_DecodeNec(IR_CH ch , uint16_t _width)
{
	/* NEC ��ʽ ��5�Σ�
		1��������  9ms�� + 4.5ms��
		2����8λ��ַ��  0=1.125ms  1=2.25ms    bit0�ȴ�
		3����8λ��ַ��  0=1.125ms  1=2.25ms
		4��8λ����      0=1.125ms  1=2.25ms
		5��8Ϊ���뷴��  0=1.125ms  1=2.25ms
	*/

loop1:	
	//bsp_LedToggle(1);		//for DEBUG �۲��Ƿ��ܹ������������ش��������ж�
	switch (g_tIR.Status[ch])
	{
		case 0:			/* 929 �ȴ���������ź�  7ms - 11ms */
			if ((_width > 700) && (_width < 1100))
			{
				g_tIR.Status[ch] = 1;
				g_tIR.s_Byte[ch] = 0;
				g_tIR.s_Bit[ch] = 0;
			}
			else if((_width > 5000))
			{
				//DEBUG("���\r\n");
				
				/*����ڼ���и���ֵ*/
				g_tIR.isRev[ch][IR_TX_SITE_LEFT] = g_tIR.isRevFilter[ch][IR_TX_SITE_LEFT];
				g_tIR.isRev[ch][IR_TX_SITE_CENTER] = g_tIR.isRevFilter[ch][IR_TX_SITE_CENTER];
				g_tIR.isRev[ch][IR_TX_SITE_RIGHT] = g_tIR.isRevFilter[ch][IR_TX_SITE_RIGHT];
				
				/*�˲���ʱ״̬����*/
				g_tIR.isRevFilter[ch][IR_TX_SITE_LEFT] = false;
				g_tIR.isRevFilter[ch][IR_TX_SITE_CENTER] = false;
				g_tIR.isRevFilter[ch][IR_TX_SITE_RIGHT] = false;
				
			}
			else
			{
				static uint8_t sss = 0;
				
				if (sss == 0)
				{
					sss = 1;
				}
				else if (sss == 1)
				{
					sss = 2;
				}				
			}
			break;

		case 1:			/* 413 �ж���������ź�  3ms - 6ms */
			if ((_width > 313) && (_width < 600))	/* ������ 4.5ms */
			{
				g_tIR.Status[ch] = 2;
			}
			else if ((_width > 150) && (_width < 250))	/* 2.25ms */
			{
				#if IR_REPEAT_SEND_EN				
					if (g_tIR.RepeatCount[ch] >= IR_REPEAT_FILTER)
					{
						bsp_PutKey(g_tIR.RxBuf[2] + IR_KEY_STRAT);	/* ������ */
					}
					else
					{
						g_tIR.RepeatCount[ch]++;
					}
				#endif
				g_tIR.Status[ch] = 0;	/* ��λ����״̬ */
			}
			else
			{
				/* �쳣���� */
				g_tIR.Status[ch] = 0;	/* ��λ����״̬ */
			}
			break;
		
		case 2:			/* �͵�ƽ�ڼ� 0.56ms */
			if ((_width > 10) && (_width < 100))
			{		
				g_tIR.Status[ch] = 3;
				g_tIR.s_LowWidth[ch] = _width;	/* ����͵�ƽ��� */
			}
			else	/* �쳣���� */
			{
				/* �쳣���� */
				g_tIR.Status[ch] = 0;	/* ��λ������״̬ */	
				goto loop1;		/* �����ж�ͬ���ź� */
			}
			break;

		case 3:			/* 85+25, 64+157 ��ʼ��������32bit */						
			g_tIR.TotalWitdh[ch] = g_tIR.s_LowWidth[ch] + _width;
			/* 0�Ŀ��Ϊ1.125ms��1�Ŀ��Ϊ2.25ms */				
			g_tIR.s_Byte[ch] <<= 1;
			if ((g_tIR.TotalWitdh[ch] > 92) && (g_tIR.TotalWitdh[ch] < 132))
			{
				;					/* bit = 0 */
			}
			else if ((g_tIR.TotalWitdh[ch] > 205) && (g_tIR.TotalWitdh[ch] < 245))
			{
				g_tIR.s_Byte[ch] += 0x01;		/* bit = 1 */
			}	
			else
			{
				/* �쳣���� */
				g_tIR.Status[ch] = 0;	/* ��λ������״̬ */	
				goto loop1;		/* �����ж�ͬ���ź� */
			}
			
			g_tIR.s_Bit[ch]++;
			if (g_tIR.s_Bit[ch] == 8)	/* ����8λ */
			{
				g_tIR.RxBuf[ch][0] = g_tIR.s_Byte[ch];
				g_tIR.s_Byte[ch] = 0;
				
//				if(ch == IR_CH3)
//					DEBUG("CH%d:%02X\r\n",ch+1,g_tIR.RxBuf[ch][0]);
				/*���·��䷶Χ*/
				if(g_tIR.RxBuf[ch][0] == IR_TX_CODE_LEFT)
				{
					g_tIR.isRevFilter[ch][IR_TX_SITE_LEFT] = true;
				}
				else if(g_tIR.RxBuf[ch][0] == IR_TX_CODE_CENTER)
				{
					g_tIR.isRevFilter[ch][IR_TX_SITE_CENTER] = true;
				}
				else if(g_tIR.RxBuf[ch][0] == IR_TX_CODE_RIGHT)
				{
					g_tIR.isRevFilter[ch][IR_TX_SITE_RIGHT] = true;
				}
				
				g_tIR.Status[ch] = 0;	/* �ȴ���һ����� */
				break;
			}
			g_tIR.Status[ch] = 2;	/* ������һ��bit */
			break;	
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_IR_TimeOutProc
*	����˵��: ̫��û���յ����½����ˣ��ָ�����ͨ������ʼ״̬
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_IR_TimeOutProc(IR_CH ch)
{
	uint32_t TIM_CH[IR_COUNT] = {TIM_Channel_1,TIM_Channel_2,TIM_Channel_3,TIM_Channel_4};
	
	/* TIM3 ������ԴƵ��10us, 655360us = 0.655ms; */
	if (g_tIR.TimeOut[ch] < 2)
	{
		if (++g_tIR.TimeOut[ch] == 2)
		{
			/* ǿ������Ϊ�½��ش��� */
			{
				TIM_ICInitTypeDef  TIM_ICInitStructure;
				
				TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/* �ȴ��½��� */
				TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
				TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* ÿ�����䶼����1�β����¼� */
				TIM_ICInitStructure.TIM_ICFilter = 0x0;	
				TIM_ICInit(TIM3, &TIM_ICInitStructure);	
				
				g_tIR.WaitFallEdge[ch] = 1;
			}

			g_tIR.Status[ch] = 0;	/* �ȴ���һ����� */
			
			/*��ʱ�䶼û�������ˣ�˵��û�к����ź�*/
			g_tIR.isRev[ch][IR_TX_SITE_LEFT] = false;
			g_tIR.isRev[ch][IR_TX_SITE_CENTER] = false;
			g_tIR.isRev[ch][IR_TX_SITE_RIGHT] = false;
			
			g_tIR.isRevFilter[ch][IR_TX_SITE_LEFT] = false;
			g_tIR.isRevFilter[ch][IR_TX_SITE_CENTER] = false;
			g_tIR.isRevFilter[ch][IR_TX_SITE_RIGHT] = false;
		}
	}
}


static void bsp_IR_GetPulseWidth(IR_CH ch)
{
	uint16_t NowCapture;
	uint16_t Width;
	uint32_t TIM_CH[IR_COUNT] = {TIM_Channel_1,TIM_Channel_2,TIM_Channel_3,TIM_Channel_4};
	
	
	g_tIR.TimeOut[ch] = 0;  /* ���㳬ʱ������ */
	
	if(ch == IR_CH1)
	{
		NowCapture = TIM_GetCapture1(TIM3);	/* ��ȡ����ļ�����ֵ��������ֵ��0-65535ѭ������ */
	}
	else if(ch == IR_CH2)
	{
		NowCapture = TIM_GetCapture2(TIM3);	/* ��ȡ����ļ�����ֵ��������ֵ��0-65535ѭ������ */
	}
	else if(ch == IR_CH3)
	{
		NowCapture = TIM_GetCapture3(TIM3);	/* ��ȡ����ļ�����ֵ��������ֵ��0-65535ѭ������ */
	}
	else if(ch == IR_CH4)
	{
		NowCapture = TIM_GetCapture4(TIM3);	/* ��ȡ����ļ�����ֵ��������ֵ��0-65535ѭ������ */
	}
	

	/* 	�л�����ļ��� */
	if (g_tIR.WaitFallEdge[ch] == 0)
	{
		TIM_ICInitTypeDef  TIM_ICInitStructure;
		
		TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/* �ȴ��½��� */
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* ÿ�����䶼����1�β����¼� */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;	
		TIM_ICInit(TIM3, &TIM_ICInitStructure);	
		
		g_tIR.WaitFallEdge[ch] = 1;
	}			
	else
	{
		TIM_ICInitTypeDef  TIM_ICInitStructure;
		
		TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		/* �ȴ������� */
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* ÿ�����䶼����1�β����¼� */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;	
		TIM_ICInit(TIM3, &TIM_ICInitStructure);	
		
		g_tIR.WaitFallEdge[ch] = 0;
	}
	
	if (NowCapture >= g_tIR.LastCapture[ch])
	{
		Width = NowCapture - g_tIR.LastCapture[ch];
	}
	else if (NowCapture < g_tIR.LastCapture[ch])	/* �������ִ���󲢷�ת */
	{
		Width = ((0xFFFF - g_tIR.LastCapture[ch]) + NowCapture);
	}			
	
	if ((g_tIR.Status[ch] == 0) && (g_tIR.LastCapture[ch] == 0))
	{
		g_tIR.LastCapture[ch] = NowCapture;
		return;
	}
			
	g_tIR.LastCapture[ch] = NowCapture;	/* ���浱ǰ�������������´μ����ֵ */
	
	bsp_IRD_DecodeNec(ch , Width);		/* ���� */	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_IR_SoftTimerInit
*	����˵��: ��ʼ�����������ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_IR_SoftTimerInit(void)
{
	g_tIR.softTimer[IR_CH1][IR_TX_SITE_LEFT]   = 0 ;
	g_tIR.softTimer[IR_CH1][IR_TX_SITE_CENTER] = 0 ;
	g_tIR.softTimer[IR_CH1][IR_TX_SITE_RIGHT]  = 0 ;
	
	g_tIR.softTimer[IR_CH2][IR_TX_SITE_LEFT]   = 0 ;
	g_tIR.softTimer[IR_CH2][IR_TX_SITE_CENTER] = 0 ;
	g_tIR.softTimer[IR_CH2][IR_TX_SITE_RIGHT]  = 0 ;
	
	g_tIR.softTimer[IR_CH3][IR_TX_SITE_LEFT]   = 0 ;
	g_tIR.softTimer[IR_CH3][IR_TX_SITE_CENTER] = 0 ;
	g_tIR.softTimer[IR_CH3][IR_TX_SITE_RIGHT]  = 0 ;
	
	g_tIR.softTimer[IR_CH4][IR_TX_SITE_LEFT]   = 0 ;
	g_tIR.softTimer[IR_CH4][IR_TX_SITE_CENTER] = 0 ;
	g_tIR.softTimer[IR_CH4][IR_TX_SITE_RIGHT]  = 0 ;
	
}


/*
*********************************************************************************************************
*	�� �� ��: TIM3_IRQHandler
*	����˵��: TIM3�жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TIM3_IRQHandler(void)
{

	/* ����ж� */
	if (TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);;
		
		/*̫��û���յ����½����ˣ��ָ�����ͨ������ʼ״̬*/
        bsp_IR_TimeOutProc(IR_CH1);
		bsp_IR_TimeOutProc(IR_CH2);
		bsp_IR_TimeOutProc(IR_CH3);
		bsp_IR_TimeOutProc(IR_CH4);
	}
	
	/* ����ͨ��3�����ж� */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		bsp_IR_GetPulseWidth(IR_CH1);
		
	}
	
	/* ����ͨ��3�����ж� */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		bsp_IR_GetPulseWidth(IR_CH2);
		
	}
	
	/* ����ͨ��3�����ж� */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		bsp_IR_GetPulseWidth(IR_CH3);
		
	}
	
	/* ����ͨ��3�����ж� */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		bsp_IR_GetPulseWidth(IR_CH4);
		
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
