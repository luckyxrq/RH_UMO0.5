#include "bsp.h"


/* IO口对应的RCC时钟，复位引脚 */
#define RCC_ALL_PIN 	(RCC_APB2Periph_GPIOF)

#define GPIO_PORT_RST  GPIOF
#define GPIO_PIN_RST   GPIO_Pin_11

static uint8_t PO_Output = 0xFF;  //直接记录电平，省去读取电平时间
static uint8_t P1_Output = 0xFF;  //直接记录电平，省去读取电平时间

static void bsp_InitRST(void);
static uint8_t bsp_CheckOk(void);
static void bsp_InitRegister(void);
static uint8_t aw_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
static uint8_t aw_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);
static bool isInitOK = false;
/*
*********************************************************************************************************
*	函 数 名: bsp_InitAW9523B
*	功能说明: 初始化拓展IO芯片。
*	形    参: 无
*	返 回 值: 1 表示正常， 0 表示不正常
*********************************************************************************************************
*/
uint8_t bsp_InitAW9523B(void)
{
	bsp_InitRST();
	bsp_InitI2C();
	
	/* 检测器件是否应答 */
	if(!bsp_CheckOk())
	{
		isInitOK = false;
		return 0;
	}
		
	bsp_InitRegister();
	
	isInitOK = true;
	
	return 1;
}

bool bsp_IsInitAW9523B_OK(void)
{
	return isInitOK;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_AWReadID
*	功能说明: 读取芯片ID
*	形    参:  无
*	返 回 值: 返回芯片ID值
*********************************************************************************************************
*/
uint8_t bsp_AWReadID(void)
{
	uint8_t id = 0 ;
	
	aw_ReadBytes(&id, AW_ID , 1);
	
	return id;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_AWReadID
*	功能说明: 读取芯片ID
*	形    参:  无
*	返 回 值: 返回芯片ID值
*********************************************************************************************************
*/
uint8_t bsp_AWReadReg(ADDR addr)
{
	uint8_t data = 0 ;
	
	aw_ReadBytes(&data, addr , 1);
	
	return data;
}


void bsp_AWSetPinVal(AW_PIN pin,AW_VAL val)
{
	
	//读取当前输出状态寄存器
	if(pin <= awP0_7)
	{
		//计算寄存器值
		if(val == AW_1)
		{
			PO_Output |= 1<<(pin%8);
		}
		else
		{
			PO_Output &= ~(1<<(pin%8));
		}
		//重新设置输出状态
		aw_WriteBytes(&PO_Output , Output_Port0 , 1);
	}
	else
	{
		//计算寄存器值
		if(val == AW_1)
		{
			P1_Output |= 1<<(pin%8);
		}
		else
		{
			P1_Output &= ~(1<<(pin%8));
		}
		//重新设置输出状态
		aw_WriteBytes(&P1_Output , Output_Port1 , 1);
	}
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_InitRST
*	功能说明: 配置拓展IO复位引脚。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitRST(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_PIN, ENABLE);


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RST;
	GPIO_Init(GPIO_PORT_RST, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayMS(2);
	GPIO_ResetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayUS(40);
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
}


/*
*********************************************************************************************************
*	函 数 名: ee_CheckOk
*	功能说明: 判断串行EERPOM是否正常
*	形    参:  无
*	返 回 值: 1 表示正常， 0 表示不正常
*********************************************************************************************************
*/
static uint8_t bsp_CheckOk(void)
{
	if (i2c_CheckDevice(AW_DEV_ADDR) == 0)
	{
		return 1;
	}
	else
	{
		/* 失败后，切记发送I2C总线停止信号 */
		i2c_Stop();
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitRegister
*	功能说明: 配置寄存器，P0全部配置为开漏输出，P1只能配置为推挽输出。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitRegister(void)
{
	uint8_t data = 0 ;
	
	data = 0x00 ;//P0全部为输出模式
	aw_WriteBytes(&data , Config_Port0 , 1);
	
	data = 0x00 ;//P1全部为输出模式
	aw_WriteBytes(&data , Config_Port1 , 1);
	
	//P1只能为推挽模式，P0可以推挽或开漏
	//0x00:P0全部开漏，0x10:P0全部推挽
	data = 0x10 ;
	aw_WriteBytes(&data , GCR , 1);
	
	//全部初始电平为高
	data = 0xFF ;
	aw_WriteBytes(&data , Output_Port0 , 1);
	aw_WriteBytes(&data , Output_Port1 , 1);
	
}


/*
*********************************************************************************************************
*	函 数 名: aw_ReadBytes
*	功能说明: 从串行EEPROM指定地址处开始读取若干数据
*	形    参:  _usAddress : 起始地址
*			 _usSize : 数据长度，单位为字节
*			 _pReadBuf : 存放读到的数据的缓冲区指针
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t aw_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;

	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */

	/* 第1步：发起I2C总线启动信号 */
	i2c_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_SendByte(AW_DEV_ADDR | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
	if (AW_ADDR_BYTES == 1)
	{
		i2c_SendByte((uint8_t)_usAddress);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}
	}
	else
	{
		i2c_SendByte(_usAddress >> 8);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}

		i2c_SendByte(_usAddress);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}
	}

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	i2c_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_SendByte(AW_DEV_ADDR | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：循环读取数据 */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = i2c_ReadByte();	/* 读1个字节 */

		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if (i != _usSize - 1)
		{
			i2c_Ack();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */
		}
		else
		{
			i2c_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
	}
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: aw_WriteBytes
*	功能说明: 向串行EEPROM指定地址写入若干数据，采用页写操作提高写入效率
*	形    参:  _usAddress : 起始地址
*			 _usSize : 数据长度，单位为字节
*			 _pWriteBuf : 存放读到的数据的缓冲区指针
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
uint8_t aw_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i = 0 ;
	uint16_t usAddr = _usAddress;
	
	for(i=0;i<_usSize;i++)
	{
		/* 第1步：发起I2C总线启动信号 */
		i2c_Start();

		/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
		i2c_SendByte(AW_DEV_ADDR | I2C_WR);	/* 此处是写指令 */

		/* 第3步：发送一个时钟，判断器件是否正确应答 */
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}
		/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
		if (AW_ADDR_BYTES == 1)
		{
			i2c_SendByte((uint8_t)usAddr);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
		}
		else
		{
			i2c_SendByte(usAddr >> 8);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}

			i2c_SendByte(usAddr);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
		}

		/* 第6步：开始写入数据 */
		i2c_SendByte(_pWriteBuf[i]);

		/* 第7步：发送ACK */
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}
		
		/* 命令执行成功，发送I2C总线停止信号 */
		i2c_Stop();
		usAddr++;	/* 地址增1 */
		
	}
	
	return 1;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 0;
}


