#include "bsp.h"

/*
	作者：平衡小车之家 
	淘宝店铺：http://shop114407458.taobao.com/
*/

/*串口发送帧缓冲区*/
static uint8_t DataScope_OutPut_Buffer[42] = {0};

/*写通道数据至 待发送帧数据缓存区*/
static void DataScope_Get_Channel_Data(float Data,uint8_t Channel);
/*发送帧数据生成函数*/
static uint8_t DataScope_Data_Generate(uint8_t Channel_Number);

/*
*********************************************************************************************************
*	函 数 名: Float2Byte
*	功能说明: 将单精度浮点数据转成4字节数据并存入指定地址，用户无需直接操作此函数 
*	形    参: target:目标单精度数据，buf:待写入数组，beg:指定从数组第几个元素开始写入
*	返 回 值: 无
*********************************************************************************************************
*/
static void Float2Byte(float *target,uint8_t *buf,uint8_t beg)
{
    uint8_t *point;
    point = (uint8_t*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}


/*
*********************************************************************************************************
*	函 数 名: DataScope_Get_Channel_Data
*	功能说明: 将待发送通道的单精度浮点数据写入发送缓冲区
*	形    参: Data：通道数据，Channel：选择通道（1-10）
*	返 回 值: 无
*********************************************************************************************************
*/
static void DataScope_Get_Channel_Data(float Data,uint8_t Channel)
{
	/*通道个数大于10或等于0，直接跳出，不执行函数*/
    if ( (Channel > 10) || (Channel == 0) ) 
		return;  
    else
    {
        switch (Channel)
        {
        case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
        case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
        case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
        case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
        case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
        case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
        case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
        case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
        case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
        case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
        }
    }	 
}


/*
*********************************************************************************************************
*	函 数 名: DataScope_Data_Generate
*	功能说明: 生成 DataScopeV1.0 能正确识别的帧格式
*	形    参: Channel_Number，需要发送的通道个数
*	返 回 值: 返回发送缓冲区数据个数，返回0表示帧格式生成失败 
*********************************************************************************************************
*/
static uint8_t DataScope_Data_Generate(uint8_t Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {	
        DataScope_OutPut_Buffer[0] = '$';  //帧头
        
        switch(Channel_Number)   
        { 
        case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
        case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
        case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
        case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
        case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
        case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
        case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
        case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
        case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
        case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
        }	 
    }
    return 0;
}


void bsp_ScopeSend(float data[10] , uint8_t chCount)
{   
		uint8_t Send_Count; //串口需要发送的数据个数
		uint8_t i; 
	
		DataScope_Get_Channel_Data(data[0], 1 ); 
		DataScope_Get_Channel_Data(data[1], 2 ); 
		DataScope_Get_Channel_Data(data[2], 3 );                
		DataScope_Get_Channel_Data(data[3], 4 );   
		DataScope_Get_Channel_Data(data[4], 5 ); 
		DataScope_Get_Channel_Data(data[5], 6 );
		DataScope_Get_Channel_Data(data[6], 7 );
		DataScope_Get_Channel_Data(data[7], 8 ); 
		DataScope_Get_Channel_Data(data[8], 9 );  
		DataScope_Get_Channel_Data(data[9], 10);
	
		Send_Count = DataScope_Data_Generate(chCount);
	
		for( i = 0 ; i < Send_Count; i++) 
		{
			#if 1
			comSendChar(COM2,DataScope_OutPut_Buffer[i]);
			#else
			while((USART1->SR&0X40)==0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
			#endif
		}
}
