#include "app_motor.h"
#include <stdlib.h>

static inline int myabs(int a) { return (a < 0) ? -a : a; }

float Velocity_Left,Velocity_Right;	//�����ٶ�(mm/s)  Wheel speed (mm/s)

#define MOTOR_IGNORE_PULSE (1300)//����  1450 25Khz   ��ֵ��Ҫ����ֹ״̬΢��   Dead zone   1450 25Khz   This value needs to be fine-tuned in static state

//�������� Filter dead zone
int PWM_Ignore(int pulse)
{
	if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
  if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
	return pulse;
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
	if(motor_left == 0)//ͣ�� stop
	{
		L_PWMA = 0;
		L_PWMB = 0;
	}
	if(motor_right == 0)
	{
		R_PWMA = 0;
		R_PWMB = 0;
	}
	
	//����  Left wheel
  if(motor_left>0)	 //ǰ��   go ahead
	{
		L_PWMB = myabs(motor_left);
		L_PWMA = 0;
	}		
	else
	{
		L_PWMB = 0;
		L_PWMA = myabs(motor_left);
	}
	
	//���� Right wheel
	if(motor_right>0) //ǰ��
	{
		R_PWMA = myabs(motor_right);
		R_PWMB = 0;
	}
	else //���� Back
	{
		R_PWMA = 0;
		R_PWMB = myabs(motor_right);	
	}

}


/**************************************************************************
Function: PWM limiting range
Input   : IN��Input  max��Maximum value  min��Minimum value
Output  : Output
�������ܣ�����PWM��ֵ 
��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
����  ֵ���޷����ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}


/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/���������� //Motor speed=Encoder reading (5ms each time) * Reading frequency/harmonics/reduction ratio/Encoder accuracy
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67/10;		//����������ٶ�=ת��*�ܳ� /10����cm //Calculate the encoder speed=rotational speed * circumference/10 and convert it to cm
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67/10;		//����������ٶ�=ת��*�ܳ� /10����cm //Calculate the encoder speed=rotational speed * circumference/10 and convert it to cm
	
}




/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle��Car inclination��voltage��Voltage
Output  : 1��abnormal��0��normal
�������ܣ��쳣�رյ��		
Input   : angleCar inclinationvoltageVoltage
Output  : 1abnormal0normal
ܣ쳣رյ		
ڲangleСǣvoltageѹ
  ֵ1쳣  0
**************************************************************************/	
uint8_t Turn_Off(float angle, float voltage)
{
	u8 temp;
	// Sécurité inclinaison (> 40°), arrêt manuel (Stop_Flag == 1) ou batterie faible (entre 5.0V et 9.6V)
	if (angle < -40.0f || angle > 40.0f || Stop_Flag == 1 || (voltage < 9.6f && voltage > 5.0f))
	{
		temp = 1;
		L_PWMA = 0;
		L_PWMB = 0;
		R_PWMA = 0;
		R_PWMB = 0;
	}
	else
	{
		temp = 0;
	}
	return temp;
}
