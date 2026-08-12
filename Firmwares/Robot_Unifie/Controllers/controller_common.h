#ifndef CONTROLLER_COMMON_H
#define CONTROLLER_COMMON_H

#include "controller_config.h"

/**
 * @file controller_common.h
 * @brief Interface commune et structure d'état pour les contrôleurs.
 */

/**
 * @struct RobotState
 * @brief Représente l'état dynamique du robot équilibreur à un instant t.
 */
typedef struct {
    float x;         /**< Position linéaire du robot (mètres) */
    float x_dot;     /**< Vitesse linéaire du robot (mètres/seconde) */
    float theta;     /**< Angle d'inclinaison pitch (radians) */
    float theta_dot; /**< Vitesse angulaire d'inclinaison pitch (radians/seconde) */
} RobotState;

/**
 * @brief Calcule la commande d'accélération ou PWM générée par le contrôleur actif.
 * @param state Structure contenant les 4 variables d'état (x, x_dot, theta, theta_dot).
 * @return Valeur de commande (accélération pour LQR ou PWM pour PID).
 */
float Controller_Compute(RobotState state);

#endif /* CONTROLLER_COMMON_H */
