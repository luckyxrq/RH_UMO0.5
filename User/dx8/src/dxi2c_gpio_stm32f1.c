#include "stm32f10x.h"
  
//�������� 1:ʹ�����ģ��I2C  
#define dx8_I2C_SCL_PIN                  GPIO_Pin_6                  /* PB.06 */
#define dx8_I2C_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define dx8_I2C_SCL_GPIO_CLK             RCC_APB2Periph_GPIOB

#define dx8_I2C_SDA_PIN                  GPIO_Pin_7                  /* PB.07 */
#define dx8_I2C_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define dx8_I2C_SDA_GPIO_CLK             RCC_APB2Periph_GPIOB

typedef unsigned char  u8;
typedef __IO uint32_t  vu32;

static __inline void TWI_SCL_0(void)        { GPIO_WriteBit(dx8_I2C_SCL_GPIO_PORT,dx8_I2C_SCL_PIN,Bit_RESET); }  
static __inline void TWI_SCL_1(void)        { GPIO_WriteBit(dx8_I2C_SCL_GPIO_PORT,dx8_I2C_SCL_PIN,Bit_SET);}  
static __inline void TWI_SDA_0(void)        { GPIO_WriteBit(dx8_I2C_SDA_GPIO_PORT,dx8_I2C_SDA_PIN,Bit_RESET); }  
static __inline void TWI_SDA_1(void)        { GPIO_WriteBit(dx8_I2C_SDA_GPIO_PORT,dx8_I2C_SDA_PIN,Bit_SET);}  
static __inline u8   TWI_SDA_STATE(void)    { return GPIO_ReadInputDataBit(dx8_I2C_SDA_GPIO_PORT,dx8_I2C_SDA_PIN); }  
  
static const u8 TWI_ACK       = 0;  
static const u8 TWI_READY     = 0;  
static const u8 TWI_NACK      = 1;  
static const u8 TWI_BUS_BUSY  = 2;  
static const u8 TWI_BUS_ERROR = 3;  
  
/******************************************************************************* 
 * ��������:TWI_Initialize                                                                      
 * ��    ��:I2C��ʼ������                                                                      
 *                                                                                
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:                                                                      
 * �޸�����:2010��6��8��                                                                     
 *******************************************************************************/  
void dx8_Init(void)  
{  
  GPIO_InitTypeDef GPIO_InitStructure;  
	
	RCC_APB2PeriphClockCmd(dx8_I2C_SCL_GPIO_CLK | dx8_I2C_SDA_GPIO_CLK, ENABLE);
	
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
  
  
  GPIO_InitStructure.GPIO_Pin = dx8_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;   
  GPIO_Init(dx8_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);  
  
   
  GPIO_InitStructure.GPIO_Pin = dx8_I2C_SDA_PIN;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
  GPIO_Init(dx8_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	
	TWI_SCL_1();
	TWI_SDA_1();
    
  //////DebugPrint("Software TWI Initializing...\n");   
} 

/******************************************************************************* 
 * ��������:TWI_Delay                                                                      
 * ��    ��:��ʱ����                                                                      
 *                                                                                
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:                                                                      
 * �޸�����:2010��6��8��                                                                     
 *******************************************************************************/  
static void TWI_NOP(void)  
{  
 vu32 i, j;  
 vu32 sum = 0;  
 i = 2;  
 while(i--)  
 {  
     for (j = 0; j < 2; j++)  
     sum += i;   
 }  
 sum = i;  
}
  
/******************************************************************************* 
 * ��������:TWI_START                                                                      
 * ��    ��:��������                                                                      
 *                                                                                
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:��                                                                      
 * ��    ��:                                                                      
 * �޸�����:2010��6��8��                                                                     
 *******************************************************************************/  
static u8 TWI_START(void)  
{   
 TWI_SDA_1();   
 TWI_NOP();  
     
 TWI_SCL_1();   
 TWI_NOP();      
  
 if(!TWI_SDA_STATE())  
 {  
  ////DebugPrint("TWI_START:BUSY\n");  
  return TWI_BUS_BUSY;  
 }  
 TWI_SDA_0();  
 TWI_NOP();  
    
 TWI_SCL_0();    
 TWI_NOP();   
  
 if(TWI_SDA_STATE())  
 {  
  ////DebugPrint("TWI_START:BUS ERROR\n");  
  return TWI_BUS_ERROR;  
 }   
   
 return TWI_READY;  
}  
  
/* --------------------------------------------------------------------------*/  
/**  
 * @Brief:  TWI_STOP  
 */  
/* --------------------------------------------------------------------------*/  
static void TWI_STOP(void)  
{  
 TWI_SDA_0();   
 TWI_NOP();  
     
 TWI_SCL_1();   
 TWI_NOP();      
  
 TWI_SDA_1();  
 TWI_NOP();  
    
 //////DebugPrint("TWI_STOP\n");     
}  
  
/* --------------------------------------------------------------------------*/  
/**  
 * @Brief:  TWI_SendACK  
 */  
/* --------------------------------------------------------------------------*/  
static void TWI_SendACK(void)  
{  
 TWI_SDA_0();  
 TWI_NOP();  
 TWI_SCL_1();  
 TWI_NOP();  
 TWI_SCL_0();   
 TWI_NOP();   
 TWI_SDA_1();  
 //////DebugPrint("TWI_SendACK\n");     
}  
  
/* --------------------------------------------------------------------------*/  
/**  
 * @Brief:  TWI_SendNACK  
 */  
/* --------------------------------------------------------------------------*/  
static void TWI_SendNACK(void)  
{  
 TWI_SDA_1();  
 TWI_NOP();  
 TWI_SCL_1();  
 TWI_NOP();  
 TWI_SCL_0();   
 TWI_NOP();  
 //////DebugPrint("TWI_SendNACK\n");      
}  
  
/* --------------------------------------------------------------------------*/  
/**  
 * @Brief:  TWI_SendByte  
 *  
 * @Param: Data 
 *  
 * @Returns:    
 */  
/* --------------------------------------------------------------------------*/  
static u8 TWI_SendByte(u8 Data)  
{  
 u8 i;  
 TWI_SCL_0();  
 for(i=0;i<8;i++)  
 {    
  //---------���ݽ���----------  
  if(Data&0x80)  
  {  
   TWI_SDA_1();  
  }  
  else  
  {  
   TWI_SDA_0();  
  }   
  Data<<=1;  
  TWI_NOP();  
  //---���ݽ�������һ����ʱ----  
    
  //----����һ��������[������]   
  TWI_SCL_1();  
  TWI_NOP();  
  TWI_SCL_0();  
  TWI_NOP();//��ʱ,��ֹSCL��û��ɵ�ʱ�ı�SDA,�Ӷ�����START/STOP�ź�  
  //---------------------------     
 }  
 //���մӻ���Ӧ��   
 TWI_SDA_1();   
 TWI_NOP();  
 TWI_SCL_1();  
 TWI_NOP();     
 if(TWI_SDA_STATE())  
 {  
  TWI_SCL_0();  
  TWI_SDA_1();
  TWI_NOP(); 	 
  //////DebugPrint("TWI_NACK!\n");  
  return TWI_NACK;  
 }  
 else  
 {  
  TWI_SCL_0();  
  TWI_SDA_1();
  TWI_NOP(); 	 
  //////DebugPrint("TWI_ACK!\n");  
  return TWI_ACK;    
 }      
}  
  
/* --------------------------------------------------------------------------*/  
/**  
 * @Brief:  TWI_ReceiveByte  
 *  
 * @Returns:    
 */  
/* --------------------------------------------------------------------------*/  
static u8 TWI_ReceiveByte(void)  
{  
 u8 i,Dat;  
 TWI_SDA_1();  
 TWI_SCL_0();   
 Dat=0;  
 for(i=0;i<8;i++)  
 {  
  TWI_SCL_1();//����ʱ��������[������],�ôӻ�׼��������   
  TWI_NOP();   
  Dat<<=1;  
  if(TWI_SDA_STATE()) //������״̬  
  {  
   Dat|=0x01;   
  }     
  TWI_SCL_0();//׼�����ٴν�������    
  TWI_NOP();//�ȴ�����׼����           
 }  
 //////DebugPrint("TWI_Dat:%x\n",Dat);  
 return Dat;  
}  

unsigned char dxif_transfer(unsigned char *buf, unsigned short len)
{
	unsigned short i;
	
	// Start
	if (TWI_READY != TWI_START()) return 1;
	
	// slave address
	if (TWI_ACK != TWI_SendByte(buf[0])) 
	{
		TWI_STOP();
		return 1;
	}
	
	// Data
	if (buf[0] & 0x01)  // i2c read
	{
		for (i = 1; i < len; i++) {
			buf[i] = TWI_ReceiveByte();
			if (i == (len - 1)) {
				TWI_SendNACK();
			}
			else {
				TWI_SendACK();
			}
		}
	}
	else  // i2c write
	{		
		for (i = 1; i < len; i++) 
		{
			if (TWI_ACK != TWI_SendByte(buf[i]))
			{
				TWI_STOP();
				return 1;
			}
		}
	}
	
	// Stop
	TWI_STOP();
	
	return 0;
}
