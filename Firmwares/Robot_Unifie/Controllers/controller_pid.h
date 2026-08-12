#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "controller_config.h"

#if TYPE_CONTROLLER == 2

#include "controller_common.h"

/**
 * @file controller_pid.h
 * @brief Module du contrôleur PID (Double boucle : PD d'angle + PI de vitesse).
 */

// Paramètres de contrôle PD de la boucle verticale (Équilibre)
extern float Balance_Kp;
extern float Balance_Kd;

// Paramètres de contrôle PI de la boucle de vitesse
extern float Velocity_Kp;
extern float Velocity_Ki;

extern int Mid_Angle;

/**
 * @brief Calcule le terme de commande d'équilibre PD.
 * @param Angle Angle mesuré (degrés)
 * @param Gyro Vitesse angulaire mesurée (degrés/s)
 * @return Valeur PWM d'équilibre
 */
int Balance_PD(float Angle, float Gyro);

/**
 * @brief Calcule le terme de commande de vitesse PI à partir des encodeurs.
 * @param encoder_left Vitesse/Impulsions encodeur gauche
 * @param encoder_right Vitesse/Impulsions encodeur droit
 * @return Valeur PWM de vitesse
 */
int Velocity_PI(int encoder_left, int encoder_right);


#endif /* TYPE_CONTROLLER == 2 */

#endif /* CONTROLLER_PID_H */
