#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

/**
 * @file controller_config.h
 * @brief Configuration du type de contrôleur pour le robot auto-équilibré.
 * 
 * Choix du type de régulateur :
 *   1 = LQR (Linear Quadratic Regulator - Retour d'état 4D)
 *   2 = PID (Double boucle : PD d'angle + PI de vitesse)
 */

#define TYPE_CONTROLLER 3

#if TYPE_CONTROLLER != 1 && TYPE_CONTROLLER != 2 && TYPE_CONTROLLER != 3
#error "TYPE_CONTROLLER doit valoir 1 (LQR) ou 2 (PID) 3 (RL-SAC)"
#endif

#endif /* CONTROLLER_CONFIG_H */
