#ifndef __BSP_POWERSWITCH_H
#define __BSP_POWERSWITCH_H

/*放在头文件，方便低功耗，不修改这几个引脚为高阻态*/
#define RCC_ALL_SW 	(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_5V_EN_CTR  GPIOE
#define GPIO_PIN_5V_EN_CTR   GPIO_Pin_15


#define GPIO_PORT_IR_POWER  GPIOF
#define GPIO_PIN_IR_POWER   GPIO_Pin_0


#define GPIO_PORT_MOTOR_POWER  GPIOA
#define GPIO_PIN_MOTOR_POWER   GPIO_Pin_9

typedef enum
{
	SW_5V_EN_CTRL = 0 ,
	SW_IR_POWER,
	SW_MOTOR_POWER
}SW_ID;


void bsp_SwOn(SW_ID sw);
void bsp_SwOff(SW_ID sw);
void bsp_InitSW(void);


#endif

