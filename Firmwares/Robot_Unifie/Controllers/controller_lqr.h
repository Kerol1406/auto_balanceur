#ifndef CONTROLLER_LQR_H
#define CONTROLLER_LQR_H

#include "controller_config.h"

#if TYPE_CONTROLLER == 1

#include "controller_common.h"

/**
 * @file controller_lqr.h
 * @brief Module du contrôleur LQR (Linear Quadratic Regulator - Retour d'état 4D).
 */

// Gains du régulateur LQR (accessibles pour réglage si besoin)
extern float K1;
extern float K2;
extern float K3;
extern float K4;

extern float Target_x_speed;
extern float Target_angle_x;

/**
 * @brief Calcul du LQR 4D
 */
float LQR_Compute(RobotState state);

#endif /* TYPE_CONTROLLER == 1 */

#endif /* CONTROLLER_LQR_H */
