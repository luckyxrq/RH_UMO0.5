#include "bsp.h"


#define RCC_ALL_SW 	(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF)

#define GPIO_PORT_5V_EN_CTR  GPIOE
#define GPIO_PIN_5V_EN_CTR   GPIO_Pin_15


#define GPIO_PORT_IR_POWER  GPIOF
#define GPIO_PIN_IR_POWER   GPIO_Pin_0


#define GPIO_PORT_MOTOR_POWER  GPIOA
#define GPIO_PIN_MOTOR_POWER   GPIO_Pin_9


/*
*********************************************************************************************************
*	函 数 名: bsp_InitSW
*	功能说明: 初始化电压使能引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitSW(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_SW, ENABLE);

	/*
		初始状态先关闭
	*/
	bsp_SwOff(SW_5V_EN_CTRL);
	bsp_SwOff(SW_IR_POWER);
	bsp_SwOff(SW_MOTOR_POWER);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_5V_EN_CTR;
	GPIO_Init(GPIO_PORT_5V_EN_CTR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	/* 开漏输出模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IR_POWER;
	GPIO_Init(GPIO_PORT_IR_POWER, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_MOTOR_POWER;
	GPIO_Init(GPIO_PORT_MOTOR_POWER, &GPIO_InitStructure);
	
}



/*
*********************************************************************************************************
*	函 数 名: bsp_SwOn
*	功能说明: 开启电压使能
*	形    参:  sw : 使能端ID
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SwOn(SW_ID sw)
{
	switch(sw)
	{
		case SW_5V_EN_CTRL:
		{
			GPIO_SetBits(GPIO_PORT_5V_EN_CTR,GPIO_PIN_5V_EN_CTR);
		}break;
		
		case SW_IR_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_IR_POWER,GPIO_PIN_IR_POWER);
		}break;
		
		case SW_MOTOR_POWER:
		{
			GPIO_SetBits(GPIO_PORT_MOTOR_POWER,GPIO_PIN_MOTOR_POWER);
		}break;
		
		default: break;
	}

}

/*
*********************************************************************************************************
*	函 数 名: bsp_SwOff
*	功能说明: 关闭电压使能
*	形    参:  sw : 使能端ID
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SwOff(SW_ID sw)
{
	switch(sw)
	{
		case SW_5V_EN_CTRL:
		{
			GPIO_ResetBits(GPIO_PORT_5V_EN_CTR,GPIO_PIN_5V_EN_CTR);
		}break;
		
		case SW_IR_POWER:
		{
			GPIO_SetBits(GPIO_PORT_IR_POWER,GPIO_PIN_IR_POWER);
		}break;
		
		case SW_MOTOR_POWER:
		{
			GPIO_ResetBits(GPIO_PORT_MOTOR_POWER,GPIO_PIN_MOTOR_POWER);
		}break;
		
		default: break;
	}

}

