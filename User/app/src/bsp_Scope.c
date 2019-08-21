#include "bsp.h"

/*
	���ߣ�ƽ��С��֮�� 
	�Ա����̣�http://shop114407458.taobao.com/
*/

/*���ڷ���֡������*/
static uint8_t DataScope_OutPut_Buffer[42] = {0};

/*дͨ�������� ������֡���ݻ�����*/
static void DataScope_Get_Channel_Data(float Data,uint8_t Channel);
/*����֡�������ɺ���*/
static uint8_t DataScope_Data_Generate(uint8_t Channel_Number);

/*
*********************************************************************************************************
*	�� �� ��: Float2Byte
*	����˵��: �������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ���û�����ֱ�Ӳ����˺��� 
*	��    ��: target:Ŀ�굥�������ݣ�buf:��д�����飬beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void Float2Byte(float *target,uint8_t *buf,uint8_t beg)
{
    uint8_t *point;
    point = (uint8_t*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}


/*
*********************************************************************************************************
*	�� �� ��: DataScope_Get_Channel_Data
*	����˵��: ��������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
*	��    ��: Data��ͨ�����ݣ�Channel��ѡ��ͨ����1-10��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DataScope_Get_Channel_Data(float Data,uint8_t Channel)
{
	/*ͨ����������10�����0��ֱ����������ִ�к���*/
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
*	�� �� ��: DataScope_Data_Generate
*	����˵��: ���� DataScopeV1.0 ����ȷʶ���֡��ʽ
*	��    ��: Channel_Number����Ҫ���͵�ͨ������
*	�� �� ֵ: ���ط��ͻ��������ݸ���������0��ʾ֡��ʽ����ʧ�� 
*********************************************************************************************************
*/
static uint8_t DataScope_Data_Generate(uint8_t Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
    else
    {	
        DataScope_OutPut_Buffer[0] = '$';  //֡ͷ
        
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
		uint8_t Send_Count; //������Ҫ���͵����ݸ���
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
