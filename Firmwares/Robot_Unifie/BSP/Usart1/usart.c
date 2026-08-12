#include "usart.h"	  

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  Add the following code to support the printf function without selecting use MicroLIB
#if 1
// Redirection de printf() pour GCC / Newlib sous STM32CubeIDE
int __io_putchar(int ch)
{
	while ((USART1->SR & 0X40) == 0);
	USART1->DR = (uint8_t) ch;
	return ch;
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}
#endif
/**************************************************************************
Function: Serial port 1 initialization
Input   : bound��Baud rate
Output  : none
�������ܣ�����1��ʼ��
��ڲ�����bound��������
����  ֵ����
**************************************************************************/
void uart_init(u32 bound)
{
  //GPIO�˿����� GPIO port settings
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��  Enable USART1, GPIOA clock
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������  Multiplexed push-pull output
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9  Initialize GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��  GPIOA.10 Initialization
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//�������� Floating Input
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10   Initialize GPIOA.10
   //USART ��ʼ������ Initial Setup

	USART_InitStructure.USART_BaudRate = bound;//���ڲ����� Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ The word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ One stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ No parity bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������ No hardware flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ Transceiver mode

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1 Initialize serial port 1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//�رմ��ڽ����ж� Disable serial port receive interrupt
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1  Enable serial port 1
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**
 * @Brief: UART1�������� UART1 sends data
 * @Note: 
 * @Parm: ch:�����͵�����  Data to be sent
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1�������� UART1 sends data
 * @Note: 
 * @Parm: BufferPtr:�����͵�����(Data to be sent)  Length:���ݳ���(Data length)
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//�����жϷ����� Serial port interrupt service function
void USART1_IRQHandler(void)
{
	uint8_t Rx1_Temp = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Rx1_Temp = USART_ReceiveData(USART1);
		USART1_Send_U8(Rx1_Temp);
	}
}


