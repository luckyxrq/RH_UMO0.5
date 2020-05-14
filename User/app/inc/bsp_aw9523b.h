#ifndef __BSP_AW9523B_H
#define __BSP_AW9523B_H

#define AW_DEV_ADDR			0xB6		/* 设备地址 */
#define AW_ADDR_BYTES		1			/* 地址字节个数 */

typedef enum
{
	AW_ID = 0x10,
	Config_Port0 = 0x04,//选择输入或输出
	Config_Port1 = 0x05,//选择输入或输出
	GCR = 0x11,//配置P0口为推挽或开漏
	Output_Port0 = 0x02,
	Output_Port1 = 0x03
}ADDR;

typedef enum
{
	awP0_0 = 0,
	awP0_1 = 1,
	awP0_2 = 2,
	awP0_3 = 3,
	awP0_4 = 4,
	awP0_5 = 5,
	awP0_6 = 6,
	awP0_7 = 7,
	
	awP1_0 = 8,
	awP1_1 = 9,
	awP1_2 = 10,
	awP1_3 = 11,
	awP1_4 = 12,
	awP1_5 = 13,
	awP1_6 = 14,
	awP1_7 = 15
	
}AW_PIN;

typedef enum
{
	AW_0 = 0,
	AW_1 = 1
}AW_VAL;

typedef enum
{
	IR0 = 0,
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	IR8,
	IR9,
}IR_SN;


uint8_t bsp_InitAW9523B(void);
uint8_t bsp_AWReadID(void);
void bsp_AWSetPinVal(AW_PIN pin,AW_VAL val);
uint8_t bsp_AWReadReg(ADDR addr);
float bsp_GetAdScanValue(void);
void bsp_DetectDeal(void);
float bsp_GetInfraredVoltageLeft(void);
float bsp_GetInfraredVoltageRight(void);
float bsp_GetInfraRedAdcVoltage(IR_SN sn);
void bsp_DetectMeasureTest(void);
bool bsp_IsInitAW9523B_OK(void);
void bsp_GetAllIrIsObstacle(bool ret[]);
#endif

