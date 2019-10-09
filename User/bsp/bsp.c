/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��(For STM32F1XX)
*	�ļ����� : bsp.c
*	��    �� : V1.0
*	˵    �� : ����Ӳ���ײ���������ģ������ļ�����Ҫ�ṩ bsp_Init()��������������á��������ÿ��c�ļ������ڿ�
*			  ͷ	��� #include "bsp.h" ���������е���������ģ�顣
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-03-01 armfly   ��ʽ����
*
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"


/*
*********************************************************************************************************
*	�� �� ��: bsp_Init
*	����˵��: ��ʼ��Ӳ���豸��ֻ��Ҫ����һ�Ρ��ú�������CPU�Ĵ���������ļĴ�������ʼ��һЩȫ�ֱ�����
*			 ȫ�ֱ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void bsp_Init(void)
{
	uint8_t ret;
	
	UNUSED(ret);
	
	/*
		����ST�̼���������ļ��Ѿ�ִ����CPUϵͳʱ�ӵĳ�ʼ�������Բ����ٴ��ظ�����ϵͳʱ�ӡ�
		�����ļ�������CPU��ʱ��Ƶ�ʡ��ڲ�Flash�����ٶȺͿ�ѡ���ⲿSRAM FSMC��ʼ����

		ϵͳʱ��ȱʡ����Ϊ72MHz�������Ҫ���ģ������޸� system_stm32f10x.c �ļ�
	*/
	
	/* ��֤ͣ��ģʽ�µ�����������������ʹ�� */
	DBGMCU_Config(DBGMCU_STOP, ENABLE);
	
	/* ���ȼ���������Ϊ4��������0-15����ռʽ���ȼ���0�������ȼ����������������ȼ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	bsp_InitDWT();
	
	bsp_InitAngle();         /* ��ʼ�������ǣ�ֻ�Ǹ�λ���ų�ʼ��������û���õ����ڴ�ӡ���ڴ��ڳ�ʼ��ǰ�渴λ�����������ǵ�һ֡���ݲ����� */
	bsp_InitPinPulse();      /* ��ʼ������ָʾ���ţ�����ָʾû��ʹ�ô��ڴ�ӡ���ڴ���֮ǰ��ʼ�� */
	bsp_InitUart(); 	     /* ��ʼ������ */
	bsp_InitLed();           /* ��ʼ��LED */
	
	bsp_InitSW();		     /* ���������������Դʹ������ */
	
	bsp_SwOn(SW_5V_EN_CTRL);
	bsp_DelayMS(1000);
	bsp_SwOn(SW_IR_POWER);
	bsp_SwOn(SW_MOTOR_POWER);
	bsp_SwOn(SW_ENCODER_POWER);
	
	/*���Գ��ʱ���ص�PA9 ��PA8�������Ǳ�������������������*/
	//bsp_SwOff(SW_MOTOR_POWER);
	
	bsp_InitKey();           /* ��ʼ������ */
	bsp_InitHardTimer();     /* ��ʼ��Ӳ����ʱ�� */
	
	bsp_InitEncoder();
	bsp_InitMotor();
	bsp_InitPid(MotorLeft);
	bsp_InitPid(MotorRight);
	
	bsp_InitCollision();     /*��ʼ����ײ��⣬��������*/
	
	bsp_InitSpeaker();		 /*��ʼ��������*/
	bsp_InitRunControl();    /*��ʼ����������״̬��*/
	
//	bsp_InitIWDG();     /*��ʼ�����Ź���һ���������Ͳ���ֹͣ*/
	/* ��ʼ��IO��չоƬ */	
	do{
		ret = bsp_InitAW9523B();		
		if(!ret) 
		{
			WARNING("AW9523B Init Error\r\n");
			bsp_DelayMS(100);
		}
	}while(!ret);
	bsp_InitDetectAct();/* IO��չоƬ��ʼ���ɹ���֮���ٳ�ʼ��������ѯɨ�� */	
	
	bsp_IRD_StartWork();
	bsp_InitCliffSW();
	
	
	
	DEBUG("��ʼ�����\r\n");
	
}


void bsp_InitFormAwaken(void)
{
	uint8_t ret;
	
	UNUSED(ret);
	
	
	
	bsp_InitAngle();         /* ��ʼ�������ǣ�ֻ�Ǹ�λ���ų�ʼ��������û���õ����ڴ�ӡ���ڴ��ڳ�ʼ��ǰ�渴λ�����������ǵ�һ֡���ݲ����� */
	bsp_InitPinPulse();      /* ��ʼ������ָʾ���ţ�����ָʾû��ʹ�ô��ڴ�ӡ���ڴ���֮ǰ��ʼ�� */
	bsp_InitUart(); 	     /* ��ʼ������ */
	bsp_InitLed();           /* ��ʼ��LED */
	bsp_InitSW();		     /* ���������������Դʹ������ */
	
	bsp_SwOn(SW_5V_EN_CTRL);
	bsp_SwOn(SW_IR_POWER);
	bsp_SwOn(SW_MOTOR_POWER);
	
	bsp_InitKey();           /* ��ʼ������ */
	bsp_InitHardTimer();     /* ��ʼ��Ӳ����ʱ�� */
	
	bsp_InitEncoder();
	bsp_InitMotor();
	bsp_InitPid(MotorLeft);
	bsp_InitPid(MotorRight);
	
	bsp_InitCollision();     /*��ʼ����ײ��⣬��������*/
	
	bsp_InitSpeaker();		 /*��ʼ��������*/
	bsp_InitRunControl();    /*��ʼ����������״̬��*/
	
	bsp_InitIWDG();     /*��ʼ�����Ź�*/
	/* ��ʼ��IO��չоƬ */	
	do{
		ret = bsp_InitAW9523B();		
		if(!ret) 
		{
			WARNING("AW9523B Init Error\r\n");
			bsp_DelayMS(100);
		}
	}while(!ret);
	bsp_InitDetectAct();/* IO��չоƬ��ʼ���ɹ���֮���ٳ�ʼ��������ѯɨ�� */	
	
	bsp_IRD_StartWork();
	bsp_InitCliffSW();
	
}



/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
