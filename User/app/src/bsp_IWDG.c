#include "bsp.h"


static void bsp_IWDG_Config(uint8_t prer,uint16_t rlr) ;


void bsp_InitIWDG(void)
{
	/*与分频数为64,重载值为625,溢出时间为1S*/
	bsp_IWDG_Config(4,625); 
}

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
static void bsp_IWDG_Config(uint8_t prer,uint16_t rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写操作
	
	IWDG_SetPrescaler(prer);  //设置IWDG预分频值:设置IWDG预分频值为64
	
	IWDG_SetReload(rlr);  //设置IWDG重装载值
	
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable();  //使能IWDG
}
//喂独立看门狗
void bsp_IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}

