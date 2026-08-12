#include "controller_pid.h"

#if TYPE_CONTROLLER == 2

#include "app_control.h"
#include "app_motor.h"

/**
 * @file controller_pid.c
 * @brief Implémentation de la double boucle PID (PD sur l'inclinaison + PI sur la vitesse).
 * 
 * Les valeurs de gains ci-dessous sont exactement celles des fichiers sources originaux (PID_Firmware).
 * Note : Dans le code d'origine, les coefficients Kp, Kd, Ki sont pré-multipliés par 100
 *        pour faciliter l'affichage et l'ajustement. Ils sont ensuite divisés par 100 dans les fonctions.
 */

// Paramètres de contrôle PD de la boucle verticale (Équilibre d'angle)
float Balance_Kp =  //TODO
float Balance_Kd = // TODO

// Paramètres de contrôle PI de la boucle de vitesse
float Velocity_Kp = //TODO
float Velocity_Ki = //TODO


/**
 * @brief Contrôle PD d'inclinaison verticale (Équilibre)
 * @param Angle Angle d'inclinaison actuel (degrés)
 * @param Gyro Vitesse angulaire (degrés/s)
 * @return Composante PWM d'équilibre
 */
int Balance_PD(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    int balance;

    Angle_bias = Mid_Angle - Angle;  // Écart à la position d'équilibre
    Gyro_bias  = 0.0f - Gyro; 

    // Calcul de la commande PD de la boucle d'équilibre
    balance = (int)(-Balance_Kp / 100.0f * Angle_bias - Gyro_bias * Balance_Kd / 100.0f);

    return balance;
}

/**
 * @brief Contrôle PI de vitesse des roues (Encodeurs)
 * @param encoder_left Mesure encodeur roue gauche
 * @param encoder_right Mesure encodeur roue droite
 * @return Composante PWM de vitesse
 */
int Velocity_PI(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias;
    static float Encoder_Integral = 0.0f;

    // Déviation de vitesse = Vitesse cible (0) - Vitesse mesurée (somme des encodeurs)
    Encoder_Least = 0.0f - (float)(encoder_left + encoder_right);

    // Filtre passe-bas du premier ordre pour lisser la variation de vitesse
    Encoder_bias *= 0.84f;
    Encoder_bias += Encoder_Least * 0.16f;

    // Intégration de l'erreur (période d'échantillonnage 5 ms)
    Encoder_Integral += Encoder_bias;

    // Saturation / Limitation de l'intégrale
    if (Encoder_Integral > 8000.0f)  Encoder_Integral = 8000.0f;
    if (Encoder_Integral < -8000.0f) Encoder_Integral = -8000.0f;

    // Calcul de la commande PI de vitesse
    velocity = -Encoder_bias * Velocity_Kp / 100.0f - Encoder_Integral * Velocity_Ki / 100.0f;

    // Remise à zéro de l'intégrale lors de la mise hors tension / sécurité (conforme à PID_Firmware)
    if (Turn_Off(Angle_Balance, battery) == 1 || Stop_Flag == 1)
    {
        Encoder_Integral = 0.0f;
    }

    return (int)velocity;
}

/**
 * @brief Point d'entrée de l'interface commune Controller_Compute en mode PID.
 * Convertit le RobotState (rad et rad/s) en unités d'angle (degrés) pour alimenter les boucles.
 */
float Controller_Compute(RobotState state)
{
    // Conversion de l'angle (rad -> degrés)
    float angle_deg = state.theta * 180.0f / 3.14159265f;
    // Vitesse angulaire gyroscope (Gyro_Balance transmis dans state.theta_dot)
    float gyro_val  = state.theta_dot;

    // Calcul de la composante d'équilibre PD
    int balance_pwm = Balance_PD(angle_deg, gyro_val);

    // Retourne le PWM d'équilibre (la composante de vitesse encodeur est ajoutée dans app_control)
    return (float)balance_pwm;
}

#endif /* TYPE_CONTROLLER == 2 */
