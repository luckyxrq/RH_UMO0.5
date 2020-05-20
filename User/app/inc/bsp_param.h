#ifndef __BSP_PARAM_H
#define __BSP_PARAM_H

#define PARAM_VER			 0x106      /* 参数版本 */
#define PARAM_SAVE_PAGE      250             /* 保存参数的页序号 */


/*
 按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节
 切记！！虽然可以在结构体中使用float和double，但是存储的时候后面的小数不准，所以建议浮点数全部扩大倍数后再存储
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
	uint32_t data4;
}
PARAM_T;
#pragma pack()


void bsp_LoadParam(void);
void bsp_SaveParam(void);
void bsp_ParamUpdateTest(void);

#endif

