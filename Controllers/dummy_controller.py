"""
Contrôleur "réflexe" de démonstration.

Ce fichier est fourni COMPLET, volontairement : il sert de MODÈLE. Tous les
autres contrôleurs du projet (PID, LQR, flou, SAC) devront respecter la même
interface, c'est-à-dire exposer une méthode :

    compute(state, target_state=None) -> float

qui reçoit l'état du robot et retourne la commande à envoyer aux moteurs.

CONVENTIONS DU PROJET (à respecter partout)
-------------------------------------------
État      : state = [x, dx, theta, dtheta]
            x      : position du robot au sol (m), positive vers la droite
            dx     : vitesse (m/s)
            theta  : inclinaison du corps par rapport à la verticale (rad),
                     positive quand le robot penche vers la droite
            dtheta : vitesse angulaire (rad/s)

Commande  : u = RAPPORT CYCLIQUE PWM, un nombre sans unité dans [-1, 1].
            Ce n'est PAS un couple : c'est le pourcentage de tension batterie
            appliqué par le pont en H, exactement ce que l'on écrira dans le
            registre du microcontrôleur. La conversion en couple (qui dépend
            de la vitesse à cause de la force contre-électromotrice) est faite
            par motor.py.

Signe     : pour rattraper une chute vers la droite (theta > 0), les roues
            doivent avancer vers la droite pour "se remettre sous" le centre
            de gravité. Il faut donc u > 0 quand theta > 0.

LOI IMPLÉMENTÉE ICI
-------------------
Un simple réflexe proportionnel-dérivé sur l'angle :

    u = Kp * theta + Kd * dtheta

Il regarde uniquement l'inclinaison, jamais la position. Résultat attendu :
le robot reste debout mais DÉRIVE lentement, car rien ne le ramène à x = 0.
C'est exactement le problème que résout la commande en cascade que vous allez
implémenter (une boucle externe de position qui fabrique l'angle cible d'une
boucle interne d'angle).
"""

import numpy as np

import config


class DummyController:
    """Contrôleur réflexe : PD sur l'angle, aucune gestion de la position."""

    def __init__(self, Kp=40.0, Kd=0.1, u_max=None, verbose=True):
        """
        Args:
            Kp (float): gain sur l'angle (rapport cyclique par radian)
            Kd (float): gain sur la vitesse angulaire (amortissement)
            u_max (float): saturation du rapport cyclique (config.PWM_MAX par défaut)
            verbose (bool): affiche la loi de commande à la création
        """
        self.Kp = Kp
        self.Kd = Kd
        self.u_max = config.PWM_MAX if u_max is None else u_max

        if verbose:
            self.display_control_law()

    def compute(self, state, target_state=None):
        """
        Calcule la commande moteur.

        Args:
            state (array-like): état courant [x, dx, theta, dtheta]
            target_state (array-like): état visé ; seul l'angle cible
                (indice 2) est utilisé ici, la position est ignorée.

        Returns:
            float: rapport cyclique PWM saturé dans [-u_max, u_max]
        """
        if target_state is None:
            target_state = np.zeros(4)

        # Erreur d'angle par rapport à la verticale demandée
        erreur_theta = state[2] - target_state[2]
        erreur_dtheta = state[3] - target_state[3]

        u = self.Kp * erreur_theta + self.Kd * erreur_dtheta

        return float(np.clip(u, -self.u_max, self.u_max))

    def display_control_law(self):
        print("\n" + "=" * 60)
        print("CONTROLEUR REFLEXE (DUMMY) - exemple de reference")
        print("=" * 60)
        print(f"u = {self.Kp:.2f} * theta + {self.Kd:.2f} * dtheta")
        print(f"Saturation : |u| <= {self.u_max:.2f} (rapport cyclique PWM)")
        print("La position x n'est PAS regulee : le robot va deriver.")
        print("=" * 60 + "\n")


if __name__ == "__main__":
    # Petite démonstration autonome : nécessite robot.py et config.py complétés.
    from robot import Robot

    controleur = DummyController()
    bot = Robot()

    state = np.array([0.0, 0.0, -0.2, 0.0])
    for i in range(int(5.0 / config.dt)):
        u = controleur.compute(state)
        state = bot.step(state, u, config.dt)

    print(f"Apres 5 s : theta = {np.degrees(state[2]):.2f} deg, x = {state[0]:.3f} m")
