"""
C'est le fichier que l'on lance pour faire tourner une simulation :

    python main.py

Il ne contient aucune logique de contrôle : il se contente de choisir la
configuration du lancement et de la passer au MainController, qui orchestre
tout le reste (création du contrôleur, boucle de simulation, courbes,
animation).

"""

import numpy as np

from Controllers.main_controller import MainController

# --- CONFIGURATION DU LANCEMENT ---

# Contrôleur utilisé pour la simulation :
#   "DUMMY" : contrôleur fourni en exemple (à utiliser pour vérifier
#             que la physique et la visualisation fonctionnent)
#   "PID"   : cascade PID (boucle position + boucle angle)
#   "LQR"   : retour d'état optimal
#   "FUZZY" : cascade de contrôleurs à logique floue
#   "SAC"   : politique apprise par renforcement (Soft Actor-Critic)
TYPE_CONTROLEUR = "FUZZY"
TYPE_CONTROLEUR = "SAC"
TYPE_CONTROLEUR = "LQR"
#TYPE_CONTROLEUR = "PID"

# True = lance l'optimiseur associé au contrôleur avant de simuler
# False = utilise les paramètres déjà enregistrés dans config.py
FAIRE_AUTOTUNING = True

# True = affiche l'animation du robot à la fin de la simulation
FAIRE_VISUALISATION = True

# Durée simulée (secondes)
sim_time = 20.0

# État initial : [position x (m), vitesse dx (m/s), angle theta (rad), vitesse angulaire dtheta (rad/s)]
# Ici le robot est lâché penché de -0.2 rad (environ -11 degrés).
state = np.array([-1.0, 0.0, -0.3, 0.0])

# État visé : robot vertical, immobile, à la position x = 0
target_state = np.zeros(4)


def main():
    print(f">>> Mode Contrôleur : {TYPE_CONTROLEUR}")

    controller = MainController(
        TYPE_CONTROLEUR,
        FAIRE_AUTOTUNING,
        FAIRE_VISUALISATION,
        sim_time,
        state,
        target_state,
    )
    
    controller.main()


if __name__ == "__main__":
    main()
