#include "IOI2C.h"



/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
�������ܣ�IIC���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_MPU6050_Init(void)
{			
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ�� Enable PB port clock
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	//�˿����� Port Configuration
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //������� Push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB  Initialize GPIOB according to the set parameters
}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : 1
�������ܣ�ģ��IIC��ʼ�ź�
��ڲ�������
����  ֵ��1
**************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     //sda����� sda line output
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������  Clamp the I2C bus and prepare to send or receive data
	return 1;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
�������ܣ�ģ��IIC�����ź�
��ڲ�������
����  ֵ����
**************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda����� sda line output
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź� Send I2C bus end signal
	delay_us(1);							   	
}

/**************************************************************************
Function: IIC wait the response signal
Input   : none
Output  : 0��No response received��1��Response received
�������ܣ�IIC�ȴ�Ӧ���ź�
��ڲ�������
����  ֵ��0��û���յ�Ӧ��1���յ�Ӧ��
**************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����   SDA is set as input
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	    Clock output 0
	return 1;  
} 

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
�������ܣ�IICӦ��
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
	
/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
�������ܣ�IIC��Ӧ��
��ڲ�������
����  ֵ����
**************************************************************************/    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**************************************************************************
Function: IIC sends a byte
Input   : txd��Byte data sent
Output  : none
�������ܣ�IIC����һ���ֽ�
��ڲ�����txd�����͵��ֽ�����
����  ֵ����
**************************************************************************/	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ��� Pull the clock low to start data transmission
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			delay_us(1);   
			IIC_SCL=1;
			delay_us(1); 
			IIC_SCL=0;	
			delay_us(1);
    }	 
} 	 
  
/**************************************************************************
Function: IIC write data to register
Input   : addr��Device address��reg��Register address��len;Number of bytes��data��Data
Output  : 0��Write successfully��1��Failed to write
�������ܣ�IICд���ݵ��Ĵ���
��ڲ�����addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����data������
����  ֵ��0���ɹ�д�룻1��û�гɹ�д��
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
Function: IIC read register data
Input   : addr��Device address��reg��Register address��len;Number of bytes��*buf��Data read out
Output  : 0��Read successfully��1��Failed to read
�������ܣ�IIC���Ĵ���������
��ڲ�����addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����*buf���������ݻ���
����  ֵ��0���ɹ�������1��û�гɹ�����
**************************************************************************/

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
Function: IIC reads a byte
Input   : ack��Send response signal or not��1��Send��0��Do not send
Output  : receive��Data read
�������ܣ�IIC��ȡһ��λ
��ڲ�����ack���Ƿ���Ӧ���źţ�1�����ͣ�0��������
����  ֵ��receive����ȡ������
**************************************************************************/ 
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ���� SDA is set as input
    for(i=0;i<8;i++ )
	 {
			IIC_SCL=0; 
			delay_us(2);
			IIC_SCL=1;
			receive<<=1;
			if(READ_SDA)receive++;   
			delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK  Send ACK
    else
        IIC_NAck();//����nACK  Send nACK
    return receive;
}

/**************************************************************************
Function: IIC reads a byte
Input   : I2C_Addr��Device IIC address��addr:Register address
Output  : res��Data read
�������ܣ���ȡָ���豸ָ���Ĵ�����һ��ֵ
��ڲ�����I2C_Addr���豸IIC��ַ��addr:�Ĵ�����ַ
����  ֵ��res����ȡ������
**************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д���� Send a write command
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ Send Address
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	 Generates a stop condition
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ	Entering receive mode		   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//����һ��ֹͣ���� Generates a stop condition

	return res;
}
 
/**************************************************************************
Function: IIC continuous reading data
Input   : dev��Target device IIC address��reg:Register address��
					length��Number of bytes��*data:The pointer where the read data will be stored
Output  : count��Number of bytes read out-1
�������ܣ�IIC����������
��ڲ�����dev��Ŀ���豸IIC��ַ��reg:�Ĵ�����ַ��length���ֽ�����
					*data:���������ݽ�Ҫ��ŵ�ָ��
����  ֵ��count�����������ֽ�����-1
**************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д���� Send a write command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ Send Address
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	 Entering receive mode
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //��ACK�Ķ����� Read data with ACK
		 else                  data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK Last byte NACK
	}
    IIC_Stop();//����һ��ֹͣ����  Generates a stop condition
    return count;
}
/**************************************************************************
Function: Writes multiple bytes to the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��length��Number of bytes��
					*data��The pointer where the read data will be stored
Output  : 1
�������ܣ�������ֽ�д��ָ���豸ָ���Ĵ���
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��length��Ҫд���ֽ�����
					*data����Ҫд�����ݵ��׵�ַ
����  ֵ��1�������Ƿ�ɹ�
**************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д���� Send a write command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ Send Address
  IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����  Generates a stop condition

    return 1; //status == 0;
}

/**************************************************************************
Function: Reads a byte of the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��*data��The pointer where the read data will be stored
Output  : 1
�������ܣ���ȡָ���豸ָ���Ĵ�����һ��ֵ
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��*data����Ҫд�����ݵ��׵�ַ
����  ֵ��1�������Ƿ�ɹ�
**************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
Function: Write a byte to the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��data��Data to be writtenwill be stored
Output  : 1
�������ܣ�д��ָ���豸ָ���Ĵ���һ���ֽ�
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��data����Ҫд������
����  ֵ��1
**************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read, modify, and write multiple bits in a byte of the specified device specified register
Input   : dev��Target device IIC address��reg��Register address��length��Number of bytes��
					bitStart��Start bit of target byte��data��Stores the value of the target byte bit to be changed
Output  : 1��success��0��fail
�������ܣ��� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitStart��Ŀ���ֽڵ���ʼλ��
					data����Ÿı�Ŀ���ֽ�λ��ֵ
����  ֵ��1���ɹ���0��ʧ��
**************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


/**************************************************************************
Function: Read, modify, and write one bit in a byte of the specified device specified register
Input   : dev��Target device IIC address��reg��Register address��
					bitNum��To modify the bitnum bit of the target byte��data��When it is 0, the target bit will be cleared, otherwise it will be set
Output  : 1��success��0��fail
�������ܣ��� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitNum��Ҫ�޸�Ŀ���ֽڵ�bitNumλ��
					data��Ϊ0ʱ��Ŀ��λ�����壬���򽫱���λ
����  ֵ��1���ɹ���0��ʧ��
**************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


