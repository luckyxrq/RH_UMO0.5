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


#define PPPPP   0xea,0x0a,0x3c,0x87,0x99,0x8c,0x43,0xaa

#define KKKKK   0xe2,0xe1,0x31,0x8a,0xf1,0x34,0x08,0xef,0x7e,0x73,0x70,0xd6,0x97,0x5b,0xe1,0x85 

#define ZZZZZ0  0xe3,0x27,0xc2,0x74,0xe1,0x48,0x4a,0xb0,0x13,0xc7,0x34,0xd0,0x4a,0x44,0x32,0xd7 

#define ZZZZZ1  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ2  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ3  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 


//------------------------------------------------------------------------------
//  Host CPU Random Number Generation
//------------------------------------------------------------------------------
void GetSoftRandom(unsigned char *random, unsigned short len)
{
  unsigned short i;
  // Strongly recommended the seed involved by system time !!!!!
  //srand((unsigned int)time(NULL) + srand_cnt++); 
  srand(srand_cnt++); 
  for (i=0; i<len; i++) random[i] = rand() % 256;
}


//------------------------------------------------------------------------------
//  PIN and Host Authentication
//------------------------------------------------------------------------------
unsigned char AuthenticationTest()
{
   unsigned char rv;
   unsigned char random[32];
   unsigned char tmpBuf1[20] = { PPPPP };
   unsigned char tmpBuf2[20] = { KKKKK };   

   // Wakeup and Reset DX8
   rv = DX8_Reset();
   if (rv) return rv;

   // PIN Authentication
   GetSoftRandom(random,32); // Generate random for verify PIN
   rv = DX8_VerifyPin(random,tmpBuf1);
   if (rv) return rv;

   // Host Authentication
   memset(tmpBuf1,0x00,20);
   GetSoftRandom(random,32);
   rv = DX8_HostAuth(random,32,tmpBuf1);
   if (rv) return rv;
   Lib_HostAuth(random,32,tmpBuf2,tmpBuf2);
   rv = memcmp(tmpBuf1,tmpBuf2,20);
   if (rv) return rv;

   // DX8 Sleep to save power
   rv = DX8_Sleep(); 
   if (rv) return rv;

   return 0;
}

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
	bsp_SwOn(SW_3V3_EN_CTRL);
	bsp_SwOn(SW_IR_POWER);
	bsp_SwOn(SW_MOTOR_POWER);
	bsp_SwOn(SW_VSLAM_POWER);
	bsp_SwOn(SW_WIFI_POWER);
	
	bsp_InitKey();           /* ��ʼ������ */
	bsp_InitHardTimer();     /* ��ʼ��Ӳ����ʱ�� */
	
	bsp_InitEncoder();
	bsp_InitMotor();
	bsp_InitPid(MotorLeft);
	bsp_InitPid(MotorRight);
	
	bsp_InitCollision();     /*��ʼ����ײ��⣬��������*/
	
	bsp_InitSpeaker();		 /*��ʼ��������*/

#if 0
	bsp_InitIWDG();     /*��ʼ�����Ź���һ���������Ͳ���ֹͣ*/
#endif

#if 0
	/* ��ʼ��IO��չоƬ */	
	do{
		ret = bsp_InitAW9523B();		
		if(!ret) 
		{
			WARNING("AW9523B Init Error\r\n");
			bsp_DelayMS(100);
		}
	}while(!ret);
#endif
	
	bsp_InitDetectAct();/* IO��չоƬ��ʼ���ɹ���֮���ٳ�ʼ��������ѯɨ�� */	
	
	bsp_IRD_StartWork();
	bsp_InitCliffSW();
	
	/*���ſ�������*/
#if 0
	bsp_SperkerPlay(Song1);
#endif
	
	
	
	{
		char *dx8Version;
		unsigned char rv;
		
		dx8_Init();
		
		/*��ȡ���ܰ汾��Ϣ*/
		dx8Version = DX8_Version();
		DEBUG("���ܰ汾��%s\r\n",dx8Version);
		
		// Authention Test
		rv = AuthenticationTest();
		if(rv)
		{
			DEBUG("δ��ͨ��������֤\r\n");
		}
		else
		{
			DEBUG("��ϲ��ͨ��������֤\r\n");
		}
		
		
	}
	
	
	
	
	/*��ӡ��ʼ����ϣ������Լ���Ƿ񱻿��Ź�������*/
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
