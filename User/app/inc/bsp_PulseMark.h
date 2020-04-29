#ifndef __BSP_PULSEMARK_H
#define __BSP_PULSEMARK_H

/* �ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_PULSE 	(RCC_APB2Periph_GPIOC)

#define GPIO_PORT_PULSE  GPIOC
#define GPIO_PIN_PULSE	 GPIO_Pin_5


/*ʾ������ƽ��IO�ڵ�ƽ�෴*/
#define M_CLIFF_PULSE_HIGH()     GPIO_SetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE) 
#define M_CLIFF_PULSE_LOW()      GPIO_ResetBits(GPIO_PORT_PULSE,GPIO_PIN_PULSE)

void bsp_InitPinPulse(void);
void bsp_CliffPulseDetect(void);
bool bsp_GetIsDangerCliff_M(void);

void bsp_StartCliffPulseTest(void);
void bsp_CliffPulseTest(void);

#endif
