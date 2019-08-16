#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H



#define M_PI 					3.141592654
#define WHEEL_LENGTH  			0.23f

#define  BB16_SOF_1				0		// 帧头1
#define  BB16_SOF_2				1		// 帧头2
#define  BB16_SOF_3				2		// 帧头3
#define  BB16_MESSAGE_ID 		3		// 消息ID
#define  BB16_LENGHT			4		// 数据长度 
#define  BB16_PAYLOAD			5		// 数据负载
#define  BB16_CKB				6		// 校验B
#define  BB16_CKA				7		// 校验A
#define  BB16_END				8		// 帧尾

#define  CMD_ID_SPEED 	        0x25	// 
#define  CMD_ID_DISTANCE  		0x35	// 
#define  CMD_ID_ANGLE  			0x45	// 





void bsp_SendReportFrame(void);
uint8_t bsp_ReveiceCmdFrame(int16_t* left_velocity,int16_t* right_velocity);

extern double DegToRad(double deg);
extern double RadToDeg(double rad);



#endif
