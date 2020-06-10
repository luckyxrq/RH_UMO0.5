/*
*********************************************************************************************************
*
*	ģ������ : ����ң�ؽ���������ģ��
*	�ļ����� : bsp_ir_decode.h
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_IR_DECODE_H
#define _BSP_IR_DECODE_H


#define IR_REPEAT_SEND_EN		0	/* ����ʹ�� */
#define IR_REPEAT_FILTER		10	/* ң����108ms ��������������, ��������1��������ط� */
#define IR_COUNT				4   /* ����Թܵĸ��� */


typedef enum
{
	IR_CH1 = 0 ,
	IR_CH2 ,
	IR_CH3 ,
	IR_CH4 ,
}IR_CH;

/*���׮���ͳ�������ֵ�����������ָ������Գ��׮��ʱ�����ۿ���������*/
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
	
	volatile uint8_t WaitFallEdge[IR_COUNT];	/* 0 ��ʾ�ȴ������أ�1��ʾ�ȴ��½��أ������л����벶���� */
	volatile uint16_t TimeOut[IR_COUNT];
	
	volatile bool isRev[IR_COUNT][3];           /*���ڱ�ʾÿ���������ı����䷶Χ*/
	volatile bool isRevFilter[IR_COUNT][3];     /*���ڱ�ʾÿ���������ı����䷶Χ*/
	volatile uint32_t softTimer[IR_COUNT][3];   /*���յ������룬������Ӧ�����ʱ�������ʱ�䵽����������յ�״̬*/
	
	volatile uint16_t s_LowWidth[IR_COUNT];               /*����decode������ʹ֮��Ϊ�̰߳�ȫ�ĺ���*/
	volatile uint8_t s_Byte[IR_COUNT];                    /*����decode������ʹ֮��Ϊ�̰߳�ȫ�ĺ���*/
	volatile uint8_t s_Bit[IR_COUNT];                     /*����decode������ʹ֮��Ϊ�̰߳�ȫ�ĺ���*/
	volatile uint16_t TotalWitdh[IR_COUNT];               /*����decode������ʹ֮��Ϊ�̰߳�ȫ�ĺ���*/
}IRD_T;

/*
**********************************************************************************************************
                                            private ���� ���û����õ��õ�
**********************************************************************************************************
*/
extern IRD_T g_tIR;



/*
**********************************************************************************************************
                                            public ���� ���û��ɵ��õ�
**********************************************************************************************************
*/

#define IR_FL_L     (g_tIR.isRev[IR_CH2][IR_TX_SITE_LEFT]   )
#define IR_FL_M     (g_tIR.isRev[IR_CH2][IR_TX_SITE_CENTER] )
#define IR_FL_R     (g_tIR.isRev[IR_CH2][IR_TX_SITE_RIGHT]  )
				  
#define IR_FR_L     (g_tIR.isRev[IR_CH1][IR_TX_SITE_LEFT]   )
#define IR_FR_M     (g_tIR.isRev[IR_CH1][IR_TX_SITE_CENTER] )
#define IR_FR_R     (g_tIR.isRev[IR_CH1][IR_TX_SITE_RIGHT]  )
				  
#define IR_SL_L     (g_tIR.isRev[IR_CH3][IR_TX_SITE_LEFT]   )
#define IR_SL_M     (g_tIR.isRev[IR_CH3][IR_TX_SITE_CENTER] )
#define IR_SL_R     (g_tIR.isRev[IR_CH3][IR_TX_SITE_RIGHT]  )
				   
#define IR_SR_L     (g_tIR.isRev[IR_CH4][IR_TX_SITE_LEFT]   )
#define IR_SR_M     (g_tIR.isRev[IR_CH4][IR_TX_SITE_CENTER] )
#define IR_SR_R     (g_tIR.isRev[IR_CH4][IR_TX_SITE_RIGHT]  )

void bsp_InitIRD(void);
void bsp_IRD_StartWork(void);
void bsp_IRD_StopWork(void);
void bsp_IR_SoftTimerTickPerMS(void);
bool bsp_IR_GetRev(IR_CH ch , IRSite site);
void bsp_PrintIR_Rev(void);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
