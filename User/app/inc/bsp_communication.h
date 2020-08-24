#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H



/*********************************************协议格式替换部分，上位机上位机统一，直接Copy 开始*************************************************/


#include <stdint.h>

/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	/*********数据部分开始***********/
	uint8_t dustBox;
	
	int16_t wheelSpeedL;
	int16_t wheelSpeedR;
	
	int32_t wheelPulseL;
	int32_t wheelPulseR;
	
	int32_t x_pos;
	int32_t y_pos;
	
	uint8_t offsiteSW;
	uint8_t collision;
	
	uint16_t cliffMV_L;
	uint16_t cliffMV_M;
	uint16_t cliffMV_R;
	
	int16_t yaw;         /*角度值*100传过去*/
	int16_t pitch;       /*角度值*100传过去*/
	int16_t roll;        /*角度值*100传过去*/
	
	uint16_t irMV[10];  /*一圈红外MV*/
	uint8_t  irRX[4][3];  /*红外接收*/

	
	uint16_t mA_wheelL;
	uint16_t mA_wheelR;
	uint16_t mA_roll;
	uint16_t mA_sideBrush;
	uint16_t mA_vacuum;
	uint16_t v_batteryVoltage;
	uint16_t mA_batteryCurrent;
	
	/*新增 2020-08-20 */
	uint32_t systick_ms;
	
	uint8_t keyPinState;    /*按键引脚的高低电平*/
	uint8_t irRxPinState;   /*红外接收管引脚的高低电平*/
	uint8_t offsitePinState;/*离地开关引脚电平*/
	uint8_t isAwIniOK;      /*前面的红外条是否初始化OK*/
	uint8_t isAngleInitOk;  /*陀螺仪是否初始化OK*/
	
	uint8_t workState;
	uint8_t workMode;
	
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	
	uint16_t strategyMajor;
	uint16_t strategyMinor;
	
	/*********数据部分结束***********/

}MCU_FRAME;
#pragma pack()


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint32_t Cliff_L;       /*跳崖传感器阈值左*/ 
	uint32_t Cliff_M;       /*跳崖传感器阈值中*/ 
	uint32_t Cliff_R;       /*跳崖传感器阈值右*/ 
	                        
	uint32_t Edge_L;        /*沿边传感器阈值左*/
	uint32_t Edge_R;        /*沿边传感器阈值右*/
	
	uint32_t ErLangShen;    /*二郎神阈值*/

}CALIBRATION;
#pragma pack()


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	int16_t speedWheelLeft;
	int16_t speedWheelRight;
	
	uint32_t tick;
	
	
}CTR_CMD;
#pragma pack()


/*这里使用联合体，方便一些自定义的参数*/
typedef union
{
    uint8_t  sw;
    uint32_t val;
    uint32_t arr[16];
	uint32_t mcu_ver;
    MCU_FRAME mcu_frame; 
	CALIBRATION calibration;
	CTR_CMD ctr_cmd;
}UNION_PARA;


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*********数据部分开始***********/
    UNION_PARA union_para;

	/*********数据部分结束***********/
	
	uint16_t crc16;
}CMD_FRAME;
#pragma pack()

extern CMD_FRAME cmd_frame_tx;
extern CMD_FRAME cmd_frame_rx;


/*********************************************协议格式替换部分，上位机上位机统一，直接Copy 结束*************************************************/











#define WHEEL_LENGTH  			0.235F
#define M_PI 					3.14F
#define MAX_ANALYSIS_LEN	    512


#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)


void bsp_ComAnalysis(void);     /*接收帧*/
void bsp_SendReportFrameWithCRC16(void);

/*新协议*/
uint8_t GetCmdStartUpload(void);

#endif
