#include "controller_lqr.h"

#if TYPE_CONTROLLER == 1

/**
 * @file controller_lqr.c
 * @brief Implémentation du régulateur LQR par retour d'état 4D pour le robot auto-équilibré.
 * 
 * Modèle d'état (4 dimensions) :
 *   - x : Position linéaire (m)
 *   - x_dot : Vitesse linéaire dx/dt (m/s)
 *   - theta : Angle d'inclinaison pitch (rad)
 *   - theta_dot : Vitesse angulaire d'inclinaison dtheta/dt (rad/s)
 */

// Coefficients de retour d'état LQR (Régulateur 4D : x, dx/dt, theta_x, dtheta_x/dt)
// Valeurs exactes et actives issues du projet d'origine fonctionnel LQR_Controller
float K1 = //TODO
float K2 = //TODO
float K3 = //TODO
float K4 = //TODO

// Valeurs d'état cibles (Consignes de référence)
float Target_x_speed = 0.0f;   // Vitesse linéaire cible (0 m/s = immobile)
float Target_angle_x = 0.0f;   // Angle cible de référence (0 rad)

/**
 * @brief Calcule l'accélération de commande LQR selon les 4 états.
 */
float LQR_Compute(RobotState state)
{
    float accel_lqr = -( K1 * state.x
                       + K2 * (state.x_dot - Target_x_speed)
                       + K3 * (state.theta - Target_angle_x)
                       + K4 * state.theta_dot );
    return accel_lqr;
}

/**
 * @brief Point d'entrée de l'interface commune Controller_Compute en mode LQR.
 */
float Controller_Compute(RobotState state)
{
    return LQR_Compute(state);
}

#endif /* TYPE_CONTROLLER == 1 */
