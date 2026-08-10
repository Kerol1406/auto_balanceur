"""
Ce fichier sert de MODÈLE. Tous les
autres contrôleurs du projet (PID, LQR, flou, SAC) devront respecter la même
interface, c'est-à-dire exposer une méthode :

    compute(state, target_state=None) -> float

qui reçoit l'état du robot et retourne la commande à envoyer aux moteurs.

CONVENTIONS DU PROJET
-------------------------------------------
État      : state = [x, dx, theta, dtheta]
            x      : position du robot au sol (m), positive vers la droite
            dx     : vitesse (m/s)
            theta  : inclinaison du corps par rapport à la verticale (rad),
                     positive quand le robot penche vers la droite
            dtheta : vitesse angulaire (rad/s)

Commande  : u = RAPPORT CYCLIQUE PWM, un nombre sans unité dans [-1, 1].
            C'est le pourcentage de tension batterie
            appliqué par le pont en H, exactement ce que l'on écrira dans le
            registre du microcontrôleur.

Signe     : pour rattraper une chute vers la droite (theta > 0), les roues
            doivent avancer vers la droite pour "se remettre sous" le centre
            de gravité. Il faut donc u > 0 quand theta > 0.
"""

import numpy as np

import config


class DummyController:

    def __init__(self, u_max=None):
        self.u_max = config.PWM_MAX if u_max is None else u_max


    def compute(self, state, target_state=None):
        """
        Calcule la commande moteur.

        Args:
            state (array-like): état courant [x, dx, theta, dtheta]
            target_state (array-like): état visé 

        Returns:
            float: rapport cyclique PWM saturé dans [-u_max, u_max]
        """
        if target_state is None:
            target_state = np.zeros(4)

        u = 0.05

        return float(np.clip(u, -self.u_max, self.u_max))
