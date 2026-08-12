#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "AllHeader.h"

#define MPU6050_INT PAin(12)   // Broche PA12 reliée à la broche d'interruption du MPU6050

// Variables globales partagées du système
extern uint8_t GET_Angle_Way;  // Algorithme d'attitude : 1 = DMP, 2 = Kalman, 3 = Complémentaire
extern float Angle_Balance;    // Angle d'inclinaison de l'équilibreur (degrés)
extern float Gyro_Balance;     // Vitesse angulaire (degrés/s)
extern int Motor_Left;         // Commande PWM du moteur gauche
extern int Motor_Right;        // Commande PWM du moteur droit
extern u8 Stop_Flag;           // Flag de sécurité : 0 = Marche, 1 = Arrêt
extern float battery;          // Tension de la batterie (V)
extern float Acceleration_Z;   // Accélération axe Z (MPU6050)
extern int Mid_Angle;          // Médiane mécanique (0°)


// Prototypes des fonctions d'acquisition et d'interruption
void EXTI15_10_IRQHandler(void);
void Get_Angle(u8 way);
void Read_MPU6050_Burst(float *gyro_x_val, float *gyro_y_val, float *gyro_z_val,
                         float *accel_x_val, float *accel_y_val, float *accel_z_val);

#endif /* __APP_CONTROL_H_ */
