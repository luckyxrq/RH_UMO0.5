/*
*********************************************************************************************************
*
*	模块名称 : 红外遥控接收器驱动模块
*	文件名称 : bsp_ir_decode.h
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_IR_DECODE_H
#define _BSP_IR_DECODE_H


#define IR_REPEAT_SEND_EN		0	/* 连发使能 */
#define IR_REPEAT_FILTER		10	/* 遥控器108ms 发持续按下脉冲, 连续按下1秒后启动重发 */
#define IR_COUNT				4   /* 红外对管的个数 */


typedef enum
{
	IR_CH1 = 0 ,
	IR_CH2 ,
	IR_CH3 ,
	IR_CH4 ,
}IR_CH;

/*充电桩发送出来的码值，这里的左右指的是面对充电桩的时候人眼看到的左右*/
typedef enum
{
	IR_TX_CODE_LEFT   = 0x27 ,
	IR_TX_CODE_CENTER = 0x38 ,
	IR_TX_CODE_RIGHT  = 0x16
}IRCode;

typedef enum
{
	IR_TX_SITE_LEFT   = 0 ,
	IR_TX_SITE_CENTER ,
	IR_TX_SITE_RIGHT  
}IRSite;

typedef struct
{
	volatile uint16_t LastCapture[IR_COUNT];
	volatile uint8_t Status[IR_COUNT];
	volatile uint8_t RxBuf[IR_COUNT][4];
	volatile uint8_t RepeatCount[IR_COUNT];
	
	volatile uint8_t WaitFallEdge[IR_COUNT];	/* 0 表示等待上升沿，1表示等待下降沿，用于切换输入捕获极性 */
	volatile uint16_t TimeOut[IR_COUNT];
	
	volatile bool isRev[IR_COUNT][3];           /*用于表示每个传感器的被辐射范围*/
	volatile uint32_t softTimer[IR_COUNT][3];   /*当收到红外码，开启相应软件定时器，如果时间到了则清除接收到状态*/
}IRD_T;


void bsp_InitIRD(void);
void bsp_IRD_StartWork(void);
void bsp_IRD_StopWork(void);
void bsp_IR_SoftTimerTickPerMS(void);
bool bsp_IR_GetRev(IR_CH ch , IRSite site);
void bsp_PrintIR_Rev(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
