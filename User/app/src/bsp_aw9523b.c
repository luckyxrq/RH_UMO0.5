#include "bsp.h"


/* IO�ڶ�Ӧ��RCCʱ�ӣ���λ���� */
#define RCC_ALL_PIN 	(RCC_APB2Periph_GPIOF)

#define GPIO_PORT_RST  GPIOF
#define GPIO_PIN_RST   GPIO_Pin_11

static uint8_t PO_Output = 0xFF;  //ֱ�Ӽ�¼��ƽ��ʡȥ��ȡ��ƽʱ��
static uint8_t P1_Output = 0xFF;  //ֱ�Ӽ�¼��ƽ��ʡȥ��ȡ��ƽʱ��

static void bsp_InitRST(void);
static uint8_t bsp_CheckOk(void);
static void bsp_InitRegister(void);
static uint8_t aw_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
static uint8_t aw_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);
static bool isInitOK = false;
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAW9523B
*	����˵��: ��ʼ����չIOоƬ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
*********************************************************************************************************
*/
uint8_t bsp_InitAW9523B(void)
{
	bsp_InitRST();
	bsp_InitI2C();
	
	/* ��������Ƿ�Ӧ�� */
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
*	�� �� ��: bsp_AWReadID
*	����˵��: ��ȡоƬID
*	��    ��:  ��
*	�� �� ֵ: ����оƬIDֵ
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
*	�� �� ��: bsp_AWReadID
*	����˵��: ��ȡоƬID
*	��    ��:  ��
*	�� �� ֵ: ����оƬIDֵ
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
	
	//��ȡ��ǰ���״̬�Ĵ���
	if(pin <= awP0_7)
	{
		//����Ĵ���ֵ
		if(val == AW_1)
		{
			PO_Output |= 1<<(pin%8);
		}
		else
		{
			PO_Output &= ~(1<<(pin%8));
		}
		//�����������״̬
		aw_WriteBytes(&PO_Output , Output_Port0 , 1);
	}
	else
	{
		//����Ĵ���ֵ
		if(val == AW_1)
		{
			P1_Output |= 1<<(pin%8);
		}
		else
		{
			P1_Output &= ~(1<<(pin%8));
		}
		//�����������״̬
		aw_WriteBytes(&P1_Output , Output_Port1 , 1);
	}
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitRST
*	����˵��: ������չIO��λ���š�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitRST(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
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
*	�� �� ��: ee_CheckOk
*	����˵��: �жϴ���EERPOM�Ƿ�����
*	��    ��:  ��
*	�� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
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
		/* ʧ�ܺ��мǷ���I2C����ֹͣ�ź� */
		i2c_Stop();
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitRegister
*	����˵��: ���üĴ�����P0ȫ������Ϊ��©�����P1ֻ������Ϊ���������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitRegister(void)
{
	uint8_t data = 0 ;
	
	data = 0x00 ;//P0ȫ��Ϊ���ģʽ
	aw_WriteBytes(&data , Config_Port0 , 1);
	
	data = 0x00 ;//P1ȫ��Ϊ���ģʽ
	aw_WriteBytes(&data , Config_Port1 , 1);
	
	//P1ֻ��Ϊ����ģʽ��P0���������©
	//0x00:P0ȫ����©��0x10:P0ȫ������
	data = 0x10 ;
	aw_WriteBytes(&data , GCR , 1);
	
	//ȫ����ʼ��ƽΪ��
	data = 0xFF ;
	aw_WriteBytes(&data , Output_Port0 , 1);
	aw_WriteBytes(&data , Output_Port1 , 1);
	
}


/*
*********************************************************************************************************
*	�� �� ��: aw_ReadBytes
*	����˵��: �Ӵ���EEPROMָ����ַ����ʼ��ȡ��������
*	��    ��:  _usAddress : ��ʼ��ַ
*			 _usSize : ���ݳ��ȣ���λΪ�ֽ�
*			 _pReadBuf : ��Ŷ��������ݵĻ�����ָ��
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t aw_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;

	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */

	/* ��1��������I2C���������ź� */
	i2c_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	i2c_SendByte(AW_DEV_ADDR | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
	if (AW_ADDR_BYTES == 1)
	{
		i2c_SendByte((uint8_t)_usAddress);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}
	}
	else
	{
		i2c_SendByte(_usAddress >> 8);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}

		i2c_SendByte(_usAddress);
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}
	}

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	i2c_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	i2c_SendByte(AW_DEV_ADDR | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9����ѭ����ȡ���� */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = i2c_ReadByte();	/* ��1���ֽ� */

		/* ÿ����1���ֽں���Ҫ����Ack�� ���һ���ֽڲ���ҪAck����Nack */
		if (i != _usSize - 1)
		{
			i2c_Ack();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */
		}
		else
		{
			i2c_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
	}
	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: aw_WriteBytes
*	����˵��: ����EEPROMָ����ַд���������ݣ�����ҳд�������д��Ч��
*	��    ��:  _usAddress : ��ʼ��ַ
*			 _usSize : ���ݳ��ȣ���λΪ�ֽ�
*			 _pWriteBuf : ��Ŷ��������ݵĻ�����ָ��
*	�� �� ֵ: 0 ��ʾʧ�ܣ�1��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t aw_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i = 0 ;
	uint16_t usAddr = _usAddress;
	
	for(i=0;i<_usSize;i++)
	{
		/* ��1��������I2C���������ź� */
		i2c_Start();

		/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
		i2c_SendByte(AW_DEV_ADDR | I2C_WR);	/* �˴���дָ�� */

		/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}
		/* ��4���������ֽڵ�ַ��24C02ֻ��256�ֽڣ����1���ֽھ͹��ˣ������24C04���ϣ���ô�˴���Ҫ���������ַ */
		if (AW_ADDR_BYTES == 1)
		{
			i2c_SendByte((uint8_t)usAddr);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
		}
		else
		{
			i2c_SendByte(usAddr >> 8);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}

			i2c_SendByte(usAddr);
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM������Ӧ�� */
			}
		}

		/* ��6������ʼд������ */
		i2c_SendByte(_pWriteBuf[i]);

		/* ��7��������ACK */
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM������Ӧ�� */
		}
		
		/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
		i2c_Stop();
		usAddr++;	/* ��ַ��1 */
		
	}
	
	return 1;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	i2c_Stop();
	return 0;
}


