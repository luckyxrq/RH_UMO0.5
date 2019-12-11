#ifndef __BSP_POWERSWITCH_H
#define __BSP_POWERSWITCH_H

/*放在头文件，方便低功耗，不修改这几个引脚为高阻态*/
#define RCC_ALL_SW 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_5V_EN_CTR  GPIOC
#define GPIO_PIN_5V_EN_CTR   GPIO_Pin_13

#define GPIO_PORT_3V3_EN_CTR  GPIOE
#define GPIO_PIN_3V3_EN_CTR   GPIO_Pin_5


#define GPIO_PORT_IR_POWER  GPIOF
#define GPIO_PIN_IR_POWER   GPIO_Pin_0


#define GPIO_PORT_MOTOR_POWER  GPIOE
#define GPIO_PIN_MOTOR_POWER   GPIO_Pin_6


#define GPIO_PORT_VSLAM_POWER  GPIOA
#define GPIO_PIN_VSLAM_POWER   GPIO_Pin_11


#define GPIO_PORT_WIFI_POWER  GPIOD
#define GPIO_PIN_WIFI_POWER   GPIO_Pin_7

typedef enum
{
	SW_5V_EN_CTRL = 0 ,
	SW_3V3_EN_CTRL,
	SW_IR_POWER,
	SW_MOTOR_POWER,
	SW_VSLAM_POWER,
	SW_WIFI_POWER,
}SW_ID;


void bsp_SwOn(SW_ID sw);
void bsp_SwOff(SW_ID sw);
void bsp_InitSW(void);


#endif

