#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "AllHeader.h"

#define PI 3.14159265							//PIԲ����  PI ��
#define Control_Frequency  200.0	//��������ȡƵ��  Encoder reading frequency
#define Diameter_67  67.0 				//����ֱ��67mm   Wheel diameter 67mm
#define Wheel_spacing  161.0         //�־� Wheelbase
#define EncoderMultiples   4.0 		//��������Ƶ��  Encoder multiples
#define Encoder_precision  11.0 	//���������� 11��  Encoder precision 11 lines
#define Reduction_Ratio  30.0			//���ٱ�30  Reduction ratio 30
#define Perimeter  210.4867 			//�ܳ�����λmm Perimeter, unit mm


void Set_Pwm(int motor_left,int motor_right);
int PWM_Limit(int IN,int max,int min);

void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
uint8_t Turn_Off(float angle, float voltage);
	
int PWM_Ignore(int pulse);

#endif

