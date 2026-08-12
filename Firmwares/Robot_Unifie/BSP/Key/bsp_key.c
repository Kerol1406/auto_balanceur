#include "bsp_key.h"

uint16_t g_key1_long_press = 0;


// �жϰ����Ƿ񱻰��£����·���KEY_PRESS���ɿ�����KEY_RELEASE
// Check if the key is pressed, return KEY_PRESS if pressed, return KEY_RELEASE if released
static uint8_t Key1_is_Press(void)
{
	if (!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
	{
		return KEY_PRESS; // ������������£��򷵻�KEY_PRESS // If the key is pressed, returns KEY_PRESS
	}
	return KEY_RELEASE;   // ����������ɿ�״̬���򷵻�KEY_RELEASE  // If the key is released, return KEY_RELEASE
}


// ��ȡ����K1�ĳ���״̬���ۼƴﵽ����ʱ�䷵��1��δ�ﵽ����0. // Read the long press status of button K1. If the long press time is reached, return 1. If not, return 0.
// timeoutΪ����ʱ�䳤�ȣ���λΪ��  // timeout is the set time length, in seconds
uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}




// ��ȡ����K1��״̬�����·���1���ɿ�����0.
// mode:����ģʽ��0������һֱ����1��1������ֻ����һ��1
//Read the status of button K1, press to return to 1, release to return to 0
//Mode: Set mode, 0: Press and hold to return 1; 1: Press once to return 1
 /*uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
		g_key1_long_press = 0;
	}
	if (key1_state == 2)
	{
		g_key1_long_press = 1;
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}*/

uint8_t Key1_State(uint8_t mode)
{
	if (Key1_is_Press() == KEY_PRESS)
	{
		delay_ms(20); // Anti-rebond 20ms
		if (Key1_is_Press() == KEY_PRESS)
		{
			return KEY_PRESS; // Le bouton est bien appuyé !
		}
	}
	return KEY_RELEASE;
}




// ��ʼ������1 // Initialize button 1
void Key1_GPIO_Init(void)
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��  Define a GPIO_InitTypeDef type structure */
	GPIO_InitTypeDef GPIO_InitStructure;
	/*���������˿ڵ�ʱ��  Enable the clock of the button port */
	RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK, ENABLE);
	//ѡ�񰴼�������  Select the pin of the button
	GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
	//���ð���������Ϊ����  Set the button pin as input
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//ʹ�ýṹ���ʼ������  Initialize buttons using structure
	GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
}


void KEYAll_GPIO_Init(void)
{
	Key1_GPIO_Init();
	
}


/*********************************************END OF FILE**********************/
