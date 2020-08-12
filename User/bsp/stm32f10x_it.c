/*
*********************************************************************************************************
*	                                  
*	ģ������ : �ж�ģ��
*	�ļ����� : stm32f10x_it.c
*	��    �� : V2.0
*	˵    �� : ���ļ�������е��жϷ�������Ϊ�˱��������˽�����õ����жϣ����ǲ����齫�жϺ����Ƶ�����
*			���ļ���
*			
*			����ֻ��Ҫ�����Ҫ���жϺ������ɡ�һ���жϺ������ǹ̶��ģ��������޸��������ļ���
*				Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup\arm\startup_stm32f10x_hd.s
*			
*			�����ļ��ǻ�������ļ�������ÿ���жϵķ���������Щ����ʹ����WEAK �ؼ��֣���ʾ�����壬�����
*			��������c�ļ����ض����˸÷��������������ͬ��������ô�����ļ����жϺ������Զ���Ч����Ҳ��
*			�����ض���ĸ�����C++�еĺ������ص��������ơ�
*				
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v0.1    2009-12-27 armfly  �������ļ���ST�̼���汾ΪV3.1.2
*		v1.0    2011-01-11 armfly  ST�̼���������V3.4.0�汾��
*		v2.0    2011-10-16 armfly  ST�̼���������V3.5.0�汾��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x_it.h"
#include "bsp_motor.h"

#define ERR_INFO "\r\nEnter HardFault_Handler, System Halt.\r\n"
#define MEM_ERR_INFO "\r\nEnter MemManage_Handler, System Halt.\r\n"
#define BUS_ERR_INFO "\r\nEnter BusFault_Handler, System Halt.\r\n"
#define USAGE_ERR_INFO "\r\nEnter UsageFault_Handler, System Halt.\r\n"
#define DEBUG_ERR_INFO "\r\nEnter DebugMon_Handler, System Halt.\r\n"

/*
*********************************************************************************************************
*	Cortex-M3 �ں��쳣�жϷ������
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: NMI_Handler
*	����˵��: ���������жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/  
void NMI_Handler(void)
{
}

/*
*********************************************************************************************************
*	�� �� ��: HardFault_Handler
*	����˵��: Ӳ��ʧЧ�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/ 
void HardFault_Handler(void)
{
#if 0
  const char *pError = ERR_INFO;
  uint8_t i;

	/*4��ͨ��ȫ������ߵ�ƽ��������ȫ�ܷɵ�ʱ��ȷ���رյ��*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
  for (i = 0; i < sizeof(ERR_INFO); i++)
  {
     USART2->DR = pError[i];
     /* �ȴ����ͽ��� */
     while ((USART2->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif	
  /* ��Ӳ��ʧЧ�쳣����ʱ������ѭ�� */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	�� �� ��: MemManage_Handler
*	����˵��: �ڴ�����쳣�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/   
void MemManage_Handler(void)
{
#if 1
  const char *pError = ERR_INFO;
  uint8_t i;

	/*4��ͨ��ȫ������ߵ�ƽ��������ȫ�ܷɵ�ʱ��ȷ���رյ��*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
  for (i = 0; i < sizeof(MEM_ERR_INFO); i++)
  {
     USART2->DR = pError[i];
     /* �ȴ����ͽ��� */
     while ((USART2->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif	
  /* ��Ӳ��ʧЧ�쳣����ʱ������ѭ�� */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	�� �� ��: BusFault_Handler
*	����˵��: ���߷����쳣�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/    
void BusFault_Handler(void)
{
#if 1
  const char *pError = ERR_INFO;
  uint8_t i;

	/*4��ͨ��ȫ������ߵ�ƽ��������ȫ�ܷɵ�ʱ��ȷ���رյ��*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
  for (i = 0; i < sizeof(BUS_ERR_INFO); i++)
  {
     USART2->DR = pError[i];
     /* �ȴ����ͽ��� */
     while ((USART2->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif	
  /* ��Ӳ��ʧЧ�쳣����ʱ������ѭ�� */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	�� �� ��: UsageFault_Handler
*	����˵��: δ�����ָ���Ƿ�״̬�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/   
void UsageFault_Handler(void)
{
#if 1
  const char *pError = ERR_INFO;
  uint8_t i;

	/*4��ͨ��ȫ������ߵ�ƽ��������ȫ�ܷɵ�ʱ��ȷ���رյ��*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
  for (i = 0; i < sizeof(USAGE_ERR_INFO); i++)
  {
     USART2->DR = pError[i];
     /* �ȴ����ͽ��� */
     while ((USART2->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif	
  /* ��Ӳ��ʧЧ�쳣����ʱ������ѭ�� */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	�� �� ��: DebugMon_Handler
*	����˵��: ���Լ������жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/   
void DebugMon_Handler(void)
{
#if 1
  const char *pError = ERR_INFO;
  uint8_t i;

	/*4��ͨ��ȫ������ߵ�ƽ��������ȫ�ܷɵ�ʱ��ȷ���رյ��*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
  for (i = 0; i < sizeof(DEBUG_ERR_INFO); i++)
  {
     USART2->DR = pError[i];
     /* �ȴ����ͽ��� */
     while ((USART2->SR & USART_FLAG_TC) == (uint16_t)RESET);
  }
#endif	
  /* ��Ӳ��ʧЧ�쳣����ʱ������ѭ�� */
  while (1)
  {
  }
}

/*
*********************************************************************************************************
*	STM32F10x�ڲ������жϷ������
*	�û��ڴ�����õ������жϷ���������Ч���жϷ���������ο������ļ�(startup_stm32f10x_xx.s)
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: PPP_IRQHandler
*	����˵��: �����жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/    
/* 
	��Ϊ�жϷ�����������;����Ӧ���йأ����õ��û�����ģ��ı���������������ڱ��ļ�չ���������Ӵ�����
	�ⲿ������������include��䡣
	
	��ˣ������Ƽ�����ط�ֻдһ��������䣬�жϷ������ı���ŵ���Ӧ���û�����ģ���С�
	����һ����ûή�ʹ����ִ��Ч�ʣ�����������Ը��ʧ���Ч�ʣ��Ӷ���ǿ�����ģ�黯���ԡ�
	
	����extern�ؼ��֣�ֱ�������õ����ⲿ�������������ļ�ͷinclude����ģ���ͷ�ļ�
extern void ppp_ISR(void);	
void PPP_IRQHandler(void)
{
	ppp_ISR();
}
*/
