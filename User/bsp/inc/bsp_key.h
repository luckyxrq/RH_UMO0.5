/*
*********************************************************************************************************
*
*	ģ������ : ��������ģ��
*	�ļ����� : bsp_key.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_KEY_H
#define __BSP_KEY_H


/*

	�����������Ӳ�������޸�GPIO����� IsKeyDown1 - IsKeyDown8 ����

	����û��İ�������С��8��������Խ�����İ���ȫ������Ϊ�͵�1������һ��������Ӱ�������
	#define KEY_COUNT    8	  ����� bsp_key.h �ļ��ж���
*/

/*
	�������߷��䣺
		K1 ��      : PE7     (�͵�ƽ��ʾ����)
		K2 ��      : PE8     (�͵�ƽ��ʾ����)
		K3 ��      : PE10    (�͵�ƽ��ʾ����)
*/

/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_KEY 	(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_K1    GPIOE
#define GPIO_PIN_K1	    GPIO_Pin_10

#define GPIO_PORT_K2    GPIOE
#define GPIO_PIN_K2	    GPIO_Pin_8

#define GPIO_PORT_K3    GPIOE
#define GPIO_PIN_K3	    GPIO_Pin_7

#define GPIO_PORT_K4    GPIOE
#define GPIO_PIN_K4	    GPIO_Pin_7

#define GPIO_PORT_K5    GPIOE
#define GPIO_PIN_K5	    GPIO_Pin_7

#define GPIO_PORT_K6    GPIOE
#define GPIO_PIN_K6	    GPIO_Pin_7

#define GPIO_PORT_K7    GPIOE
#define GPIO_PIN_K7	    GPIO_Pin_7

#define GPIO_PORT_K8    GPIOE
#define GPIO_PIN_K8	    GPIO_Pin_7


#define KEY_COUNT    10	   					/* ��������, 8�������� + 2����ϼ� */

/* ����Ӧ�ó���Ĺ��������������� */
#define KEY_DOWN_POWER	KEY_3_DOWN
#define KEY_UP_POWER	KEY_3_UP
#define KEY_LONG_POWER	KEY_3_LONG

#define KEY_DOWN_CHARGE	KEY_1_DOWN
#define KEY_UP_CHARGE	KEY_1_UP
#define KEY_LONG_CHARGE	KEY_1_LONG

#define KEY_DOWN_CLEAN	KEY_2_DOWN
#define KEY_UP_CLEAN	KEY_2_UP
#define KEY_LONG_CLEAN	KEY_2_LONG

#define JOY_DOWN_U		KEY_4_DOWN		/* �� */
#define JOY_UP_U		KEY_4_UP
#define JOY_LONG_U		KEY_4_LONG

#define JOY_DOWN_D		KEY_5_DOWN		/* �� */
#define JOY_UP_D		KEY_5_UP
#define JOY_LONG_D		KEY_5_LONG

#define JOY_DOWN_L		KEY_6_DOWN		/* �� */
#define JOY_UP_L		KEY_6_UP
#define JOY_LONG_L		KEY_6_LONG

#define JOY_DOWN_R		KEY_7_DOWN		/* �� */
#define JOY_UP_R		KEY_7_UP
#define JOY_LONG_R		KEY_7_LONG

#define JOY_DOWN_OK		KEY_8_DOWN		/* ok */
#define JOY_UP_OK		KEY_8_UP
#define JOY_LONG_OK		KEY_8_LONG

#define SYS_DOWN_K1K2	KEY_9_DOWN		/* K1 K2 ��ϼ� */
#define SYS_UP_K1K2	    KEY_9_UP
#define SYS_LONG_K1K2	KEY_9_LONG

#define SYS_DOWN_K2K3	KEY_10_DOWN		/* K2 K3 ��ϼ� */
#define SYS_UP_K2K3  	KEY_10_UP
#define SYS_LONG_K2K3	KEY_10_LONG
     

/* ����ID, ��Ҫ����bsp_KeyState()��������ڲ��� */
typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3,
	KID_JOY_U,
	KID_JOY_D,
	KID_JOY_L,
	KID_JOY_R,
	KID_JOY_OK,
	
	KID_K9,
	KID_K10
}KEY_ID_E;

/*
	�����˲�ʱ��50ms, ��λ10ms��
	ֻ��������⵽50ms״̬�������Ϊ��Ч����������Ͱ��������¼�
	��ʹ������·����Ӳ���˲������˲�����Ҳ���Ա�֤�ɿ��ؼ�⵽�����¼�
*/
#define KEY_FILTER_TIME   6
#define KEY_LONG_TIME     50			/* ��λ10ms�� ����1�룬��Ϊ�����¼� */

/*
	ÿ��������Ӧ1��ȫ�ֵĽṹ�������
*/
typedef struct
{
	/* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */
	uint8_t (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */

	uint8_t  Count;			/* �˲��������� */
	uint16_t LongCount;		/* ���������� */
	uint16_t LongTime;		/* �������³���ʱ��, 0��ʾ����ⳤ�� */
	uint8_t  State;			/* ������ǰ״̬�����»��ǵ��� */
	uint8_t  RepeatSpeed;	/* ������������ */
	uint8_t  RepeatCount;	/* �������������� */
}KEY_T;

/*
	�����ֵ����, ���밴���´���ʱÿ�����İ��¡�����ͳ����¼�

	�Ƽ�ʹ��enum, ����#define��ԭ��
	(1) ����������ֵ,�������˳��ʹ���뿴���������
	(2) �������ɰ����Ǳ����ֵ�ظ���
*/
typedef enum
{
	KEY_NONE = 0,			/* 0 ��ʾ�����¼� */

	KEY_1_DOWN,				/* 1������ */
	KEY_1_UP,				/* 1������ */
	KEY_1_LONG,				/* 1������ */

	KEY_2_DOWN,				/* 2������ */
	KEY_2_UP,				/* 2������ */
	KEY_2_LONG,				/* 2������ */

	KEY_3_DOWN,				/* 3������ */
	KEY_3_UP,				/* 3������ */
	KEY_3_LONG,				/* 3������ */

	KEY_4_DOWN,				/* 4������ */
	KEY_4_UP,				/* 4������ */
	KEY_4_LONG,				/* 4������ */

	KEY_5_DOWN,				/* 5������ */
	KEY_5_UP,				/* 5������ */
	KEY_5_LONG,				/* 5������ */

	KEY_6_DOWN,				/* 6������ */
	KEY_6_UP,				/* 6������ */
	KEY_6_LONG,				/* 6������ */

	KEY_7_DOWN,				/* 7������ */
	KEY_7_UP,				/* 7������ */
	KEY_7_LONG,				/* 7������ */

	KEY_8_DOWN,				/* 8������ */
	KEY_8_UP,				/* 8������ */
	KEY_8_LONG,				/* 8������ */

	/* ��ϼ� */
	KEY_9_DOWN,				/* 9������ */
	KEY_9_UP,				/* 9������ */
	KEY_9_LONG,				/* 9������ */

	KEY_10_DOWN,			/* 10������ */
	KEY_10_UP,				/* 10������ */
	KEY_10_LONG,			/* 10������ */
	
	KEY_WIFI_OPEN_CLEAN_CAR,
	KEY_WIFI_CLOSE_CLEAN_CAR,
	
	KEY_WIFI_DIR_FRONT,
	KEY_WIFI_DIR_BACK,
	KEY_WIFI_DIR_LEFT,
	KEY_WIFI_DIR_RIGHT,
	
	KEY_WIFI_EDGE,
}KEY_ENUM;

/* ����FIFO�õ����� */
#define KEY_FIFO_SIZE	10
typedef struct
{
	uint8_t Buf[KEY_FIFO_SIZE];		/* ��ֵ������ */
	uint8_t Read;					/* ��������ָ��1 */
	uint8_t Write;					/* ������дָ�� */
	uint8_t Read2;					/* ��������ָ��2 */
}KEY_FIFO_T;

/* �Զ�����չ��� */
#define MAX_KEY_COUNT      3

typedef enum
{
	KEY_CLEAN = 0 ,
	KEY_POWER,
	KEY_CHARGE
}KEY_SN;


/* ���ⲿ���õĺ������� */
void bsp_InitKey(void);
void bsp_KeyScan(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKey2(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed);
void bsp_ClearKey(void);

/* �Զ�����չ��� */
void  bsp_SetIsStartKeyProc(bool val);
bool bsp_GetIsStartKeyProc(void);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
