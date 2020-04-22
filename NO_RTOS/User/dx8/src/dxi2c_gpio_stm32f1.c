#include "bsp.h"



#define PPPPP   0xea,0x0a,0x3c,0x87,0x99,0x8c,0x43,0xaa

#define KKKKK   0xe2,0xe1,0x31,0x8a,0xf1,0x34,0x08,0xef,0x7e,0x73,0x70,0xd6,0x97,0x5b,0xe1,0x85 

#define ZZZZZ0  0xe3,0x27,0xc2,0x74,0xe1,0x48,0x4a,0xb0,0x13,0xc7,0x34,0xd0,0x4a,0x44,0x32,0xd7 

#define ZZZZZ1  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ2  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ3  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 


//条件编译 1:使用软件模拟I2C  
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

static void GetSoftRandom(unsigned char *random, unsigned short len);

/******************************************************************************* 
 * 函数名称:TWI_Initialize                                                                      
 * 描    述:I2C初始化函数                                                                      
 *                                                                                
 * 输    入:无                                                                      
 * 输    出:无                                                                      
 * 返    回:无                                                                      
 * 作    者:                                                                      
 * 修改日期:2010年6月8日                                                                     
 *******************************************************************************/  
void bsp_InitDX8_(void)  
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

//------------------------------------------------------------------------------
//  返回0表示通过加密认证
//------------------------------------------------------------------------------
unsigned char bsp_Authentication(void)
{
    unsigned char rv;
    unsigned char random[32];
    unsigned char tmpBuf1[20] = { PPPPP };
    unsigned char tmpBuf2[20] = { KKKKK };   
    
    // Wakeup and Reset DX8
    rv = DX8_Reset();
    if (rv) return rv;
    
    // PIN Authentication
    GetSoftRandom(random,32); // Generate random for verify PIN
    rv = DX8_VerifyPin(random,tmpBuf1);
    if (rv) return rv;
    
    // Host Authentication
    memset(tmpBuf1,0x00,20);
    GetSoftRandom(random,32);
    rv = DX8_HostAuth(random,32,tmpBuf1);
    if (rv) return rv;
    Lib_HostAuth(random,32,tmpBuf2,tmpBuf2);
    rv = memcmp(tmpBuf1,tmpBuf2,20);
    if (rv) return rv;
    
    // DX8 Sleep to save power
    rv = DX8_Sleep(); 
    if (rv) return rv;
    
    return 0;
}

/******************************************************************************* 
 * 函数名称:TWI_Delay                                                                      
 * 描    述:延时函数                                                                      
 *                                                                                
 * 输    入:无                                                                      
 * 输    出:无                                                                      
 * 返    回:无                                                                      
 * 作    者:                                                                      
 * 修改日期:2010年6月8日                                                                     
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
 * 函数名称:TWI_START                                                                      
 * 描    述:发送启动                                                                      
 *                                                                                
 * 输    入:无                                                                      
 * 输    出:无                                                                      
 * 返    回:无                                                                      
 * 作    者:                                                                      
 * 修改日期:2010年6月8日                                                                     
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
        //---------数据建立----------  
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
        //---数据建立保持一定延时----  
        
        //----产生一个上升沿[正脉冲]   
        TWI_SCL_1();  
        TWI_NOP();  
        TWI_SCL_0();  
        TWI_NOP();//延时,防止SCL还没变成低时改变SDA,从而产生START/STOP信号  
        //---------------------------     
    }  
    //接收从机的应答   
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
        TWI_SCL_1();//产生时钟上升沿[正脉冲],让从机准备好数据   
        TWI_NOP();   
        Dat<<=1;  
        if(TWI_SDA_STATE()) //读引脚状态  
        {  
            Dat|=0x01;   
        }     
        TWI_SCL_0();//准备好再次接收数据    
        TWI_NOP();//等待数据准备好           
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




//------------------------------------------------------------------------------
//  Host CPU Random Number Generation
//------------------------------------------------------------------------------
static void GetSoftRandom(unsigned char *random, unsigned short len)
{
    unsigned short i;
    // Strongly recommended the seed involved by system time !!!!!
    //srand((unsigned int)time(NULL) + srand_cnt++); 
    srand(srand_cnt++); 
    for (i=0; i<len; i++) random[i] = rand() % 256;
}









