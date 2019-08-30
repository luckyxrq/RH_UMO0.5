/*
*********************************************************************************************************
*
*	模块名称 : 红外遥控接收器驱动模块
*	文件名称 : bsp_ir_decode.c
*	版    本 : V1.0
*	说    明 : 红外遥控接收的红外信号送入CPU的 PB0/TIM3_CH3.  本驱动程序使用TIM3_CH3通道的输入捕获功能来
*				协助解码。 
*				TIM3不支持双边沿触发, 因此需要在中断中切换极性。
*				除TIM6和TIM7之外的定时器都只能采用上升沿或者下降沿捕捉而不能采用双边沿捕捉.
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2015-07-11   armfly  正式发布
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

#define IR_REPEAT_SEND_EN		0	/* 连发使能 */
#define IR_REPEAT_FILTER		10	/* 遥控器108ms 发持续按下脉冲, 连续按下1秒后启动重发 */
#define IR_COUNT				4   /* 红外对管的个数 */

/* 定义GPIO端口 */
#define RCC_IRD		RCC_APB2Periph_GPIOC
#define PORT_IRD	GPIOC
#define PIN_IRD		(GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)

typedef enum
{
	IR_CH1 = 0 ,
	IR_CH2 ,
	IR_CH3 ,
	IR_CH4 ,
}IR_CH;

typedef struct
{
	volatile uint16_t LastCapture[IR_COUNT];
	volatile uint8_t Status[IR_COUNT];
	volatile uint8_t RxBuf[IR_COUNT][4];
	volatile uint8_t RepeatCount[IR_COUNT];
	
	volatile uint8_t WaitFallEdge[IR_COUNT];	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
	volatile uint16_t TimeOut[IR_COUNT];
}IRD_T;

static IRD_T g_tIR;

/*
*********************************************************************************************************
*	函 数 名: IRD_StartWork
*	功能说明: 配置TIM，开始解码
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_IRD_StartWork(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue;
	
	/* 时钟，重映射 */
	RCC_APB2PeriphClockCmd(RCC_IRD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/* 配置为输入引脚 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 输入模式 */
	GPIO_InitStructure.GPIO_Pin = PIN_IRD;
	GPIO_Init(PORT_IRD, &GPIO_InitStructure);	
	
	/* 定时器3中断分组 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 设置分频为, 捕获计数器值的单位正好是 10us, 方便脉宽比较。 */
	PrescalerValue = 72000000/100000 - 1;
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/*输入捕获参数配置*/
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/*对于TIM3，TIM_ICPolarity_BothEdge不起作用，自己切换上下降沿*/ 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* 每次跳变都产生1次捕获事件 */
	TIM_ICInitStructure.TIM_ICFilter = 0x0;	
	
	/*每个通道*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	/*配置溢出中断和输入捕获中断*/
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	/* 溢出中断使能，用于超时同步处理 */
	
	/* 使能定时器 */
	TIM_Cmd(TIM3, ENABLE);

	/*初始状态*/
	g_tIR.LastCapture[IR_CH1] = 0;	
	g_tIR.Status[IR_CH1] = 0;
	g_tIR.WaitFallEdge[IR_CH1] = 1;	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
	
	g_tIR.LastCapture[IR_CH2] = 0;	
	g_tIR.Status[IR_CH2] = 0;
	g_tIR.WaitFallEdge[IR_CH2] = 1;	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
	
	g_tIR.LastCapture[IR_CH3] = 0;	
	g_tIR.Status[IR_CH3] = 0;
	g_tIR.WaitFallEdge[IR_CH3] = 1;	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
	
	g_tIR.LastCapture[IR_CH4] = 0;	
	g_tIR.Status[IR_CH4] = 0;
	g_tIR.WaitFallEdge[IR_CH4] = 1;	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
}

/*
*********************************************************************************************************
*	函 数 名: IRD_StopWork
*	功能说明: 停止红外解码
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: IRD_DecodeNec
*	功能说明: 按照NEC编码格式实时解码
*	形    参: _width 脉冲宽度，单位 10us
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_IRD_DecodeNec(IR_CH ch , uint16_t _width)
{
	static uint16_t s_LowWidth;
	static uint8_t s_Byte;
	static uint8_t s_Bit;
	uint16_t TotalWitdh;
	
	/* NEC 格式 （5段）
		1、引导码  9ms低 + 4.5ms高
		2、低8位地址码  0=1.125ms  1=2.25ms    bit0先传
		3、高8位地址码  0=1.125ms  1=2.25ms
		4、8位数据      0=1.125ms  1=2.25ms
		5、8为数码反码  0=1.125ms  1=2.25ms
	*/

loop1:	
	//bsp_LedToggle(1);		//for DEBUG 观测是否能够给在两个边沿触发捕获中断
	switch (g_tIR.Status[ch])
	{
		case 0:			/* 929 等待引导码低信号  7ms - 11ms */
			if ((_width > 700) && (_width < 1100))
			{
				g_tIR.Status[ch] = 1;
				s_Byte = 0;
				s_Bit = 0;
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

		case 1:			/* 413 判断引导码高信号  3ms - 6ms */
			if ((_width > 313) && (_width < 600))	/* 引导码 4.5ms */
			{
				g_tIR.Status[ch] = 2;
			}
			else if ((_width > 150) && (_width < 250))	/* 2.25ms */
			{
				#if IR_REPEAT_SEND_EN				
					if (g_tIR.RepeatCount[ch] >= IR_REPEAT_FILTER)
					{
						bsp_PutKey(g_tIR.RxBuf[2] + IR_KEY_STRAT);	/* 连发码 */
					}
					else
					{
						g_tIR.RepeatCount[ch]++;
					}
				#endif
				g_tIR.Status[ch] = 0;	/* 复位解码状态 */
			}
			else
			{
				/* 异常脉宽 */
				g_tIR.Status[ch] = 0;	/* 复位解码状态 */
			}
			break;
		
		case 2:			/* 低电平期间 0.56ms */
			if ((_width > 10) && (_width < 100))
			{		
				g_tIR.Status[ch] = 3;
				s_LowWidth = _width;	/* 保存低电平宽度 */
			}
			else	/* 异常脉宽 */
			{
				/* 异常脉宽 */
				g_tIR.Status[ch] = 0;	/* 复位解码器状态 */	
				goto loop1;		/* 继续判断同步信号 */
			}
			break;

		case 3:			/* 85+25, 64+157 开始连续解码32bit */						
			TotalWitdh = s_LowWidth + _width;
			/* 0的宽度为1.125ms，1的宽度为2.25ms */				
			s_Byte <<= 1;
			if ((TotalWitdh > 92) && (TotalWitdh < 132))
			{
				;					/* bit = 0 */
			}
			else if ((TotalWitdh > 205) && (TotalWitdh < 245))
			{
				s_Byte += 0x01;		/* bit = 1 */
			}	
			else
			{
				/* 异常脉宽 */
				g_tIR.Status[ch] = 0;	/* 复位解码器状态 */	
				goto loop1;		/* 继续判断同步信号 */
			}
			
			s_Bit++;
			if (s_Bit == 8)	/* 收齐8位 */
			{
				g_tIR.RxBuf[ch][0] = s_Byte;
				s_Byte = 0;
				
				DEBUG("CH%d:%02X\r\n",ch,g_tIR.RxBuf[ch][0]);
				
				g_tIR.Status[ch] = 0;	/* 等待下一组编码 */
				break;
			}
			g_tIR.Status[ch] = 2;	/* 继续下一个bit */
			break;	
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_IR_TimeOutProc
*	功能说明: 太久没有收到上下降沿了，恢复各个通道到初始状态
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_IR_TimeOutProc(IR_CH ch)
{
	uint32_t TIM_CH[IR_COUNT] = {TIM_Channel_1,TIM_Channel_2,TIM_Channel_3,TIM_Channel_4};
	
	/* TIM3 计数器源频率10us, 655360us = 0.655ms; */
	if (g_tIR.TimeOut[ch] < 2)
	{
		if (++g_tIR.TimeOut[ch] == 2)
		{
			/* 强制设置为下降沿触发 */
			{
				TIM_ICInitTypeDef  TIM_ICInitStructure;
				
				TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/* 等待下降沿 */
				TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
				TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* 每次跳变都产生1次捕获事件 */
				TIM_ICInitStructure.TIM_ICFilter = 0x0;	
				TIM_ICInit(TIM3, &TIM_ICInitStructure);	
				
				g_tIR.WaitFallEdge[ch] = 1;
			}

			g_tIR.Status[ch] = 0;	/* 等待下一组编码 */
		}
	}
}


static void bsp_IR_GetPulseWidth(IR_CH ch)
{
	uint16_t NowCapture;
	uint16_t Width;
	uint32_t TIM_CH[IR_COUNT] = {TIM_Channel_1,TIM_Channel_2,TIM_Channel_3,TIM_Channel_4};
	
	
	g_tIR.TimeOut[ch] = 0;  /* 清零超时计数器 */
	
	if(ch == IR_CH1)
	{
		NowCapture = TIM_GetCapture1(TIM3);	/* 读取捕获的计数器值，计数器值从0-65535循环计数 */
	}
	else if(ch == IR_CH2)
	{
		NowCapture = TIM_GetCapture2(TIM3);	/* 读取捕获的计数器值，计数器值从0-65535循环计数 */
	}
	else if(ch == IR_CH3)
	{
		NowCapture = TIM_GetCapture3(TIM3);	/* 读取捕获的计数器值，计数器值从0-65535循环计数 */
	}
	else if(ch == IR_CH4)
	{
		NowCapture = TIM_GetCapture4(TIM3);	/* 读取捕获的计数器值，计数器值从0-65535循环计数 */
	}
	

	/* 	切换捕获的极性 */
	if (g_tIR.WaitFallEdge[ch] == 0)
	{
		TIM_ICInitTypeDef  TIM_ICInitStructure;
		
		TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	/* 等待下降沿 */
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* 每次跳变都产生1次捕获事件 */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;	
		TIM_ICInit(TIM3, &TIM_ICInitStructure);	
		
		g_tIR.WaitFallEdge[ch] = 1;
	}			
	else
	{
		TIM_ICInitTypeDef  TIM_ICInitStructure;
		
		TIM_ICInitStructure.TIM_Channel = TIM_CH[ch];
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		/* 等待上升沿 */
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			/* 每次跳变都产生1次捕获事件 */
		TIM_ICInitStructure.TIM_ICFilter = 0x0;	
		TIM_ICInit(TIM3, &TIM_ICInitStructure);	
		
		g_tIR.WaitFallEdge[ch] = 0;
	}
	
	if (NowCapture >= g_tIR.LastCapture[ch])
	{
		Width = NowCapture - g_tIR.LastCapture[ch];
	}
	else if (NowCapture < g_tIR.LastCapture[ch])	/* 计数器抵达最大并翻转 */
	{
		Width = ((0xFFFF - g_tIR.LastCapture[ch]) + NowCapture);
	}			
	
	if ((g_tIR.Status[ch] == 0) && (g_tIR.LastCapture[ch] == 0))
	{
		g_tIR.LastCapture[ch] = NowCapture;
		return;
	}
			
	g_tIR.LastCapture[ch] = NowCapture;	/* 保存当前计数器，用于下次计算差值 */
	
	bsp_IRD_DecodeNec(ch , Width);		/* 解码 */	
}


/*
*********************************************************************************************************
*	函 数 名: TIM3_IRQHandler
*	功能说明: TIM3中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM3_IRQHandler(void)
{

	/* 溢出中断 */
	if (TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);;
		
		/*太久没有收到上下降沿了，恢复各个通道到初始状态*/
        bsp_IR_TimeOutProc(IR_CH1);
		bsp_IR_TimeOutProc(IR_CH2);
		bsp_IR_TimeOutProc(IR_CH3);
		bsp_IR_TimeOutProc(IR_CH4);
	}
	
	/* 输入通道3捕获中断 */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		bsp_IR_GetPulseWidth(IR_CH1);
		
	}
	
	/* 输入通道3捕获中断 */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		bsp_IR_GetPulseWidth(IR_CH2);
		
	}
	
	/* 输入通道3捕获中断 */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		bsp_IR_GetPulseWidth(IR_CH3);
		
	}
	
	/* 输入通道3捕获中断 */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		bsp_IR_GetPulseWidth(IR_CH4);
		
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
