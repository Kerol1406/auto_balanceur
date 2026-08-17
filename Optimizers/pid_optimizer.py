"""
Réglage automatique de la cascade PID par la méthode "twiddle".

TWIDDLE
---------------------------------------------------
Algorithme très simple, sans dérivée :

    pour chaque paramètre p_i :
        essayer p_i + dp_i
        si c'est mieux           -> garder, et AGRANDIR le pas (dp_i *= 1.1)
        sinon essayer p_i - dp_i
            si c'est mieux       -> garder, et agrandir le pas
            sinon                -> revenir en arrière et RÉDUIRE le pas (dp_i *= 0.9)
    recommencer tant que la somme des pas dépasse une tolérance

DEUX PIÈGES À ÉVITER
--------------------------------------------------------------------
  1. Optimiser les deux boucles SÉPARÉMENT. Elles interagissent : un gain de
     position excellent avec une boucle d'angle donnée peut déstabiliser une
     autre. On règle donc les cinq gains ensemble.

  2. Évaluer sur UNE SEULE trajectoire. L'algorithme apprend alors par cœur ce
     cas précis et donne des gains médiocres partout ailleurs. On évalue sur
     plusieurs états initiaux et on retient le PIRE des coûts.

À COMPLÉTER : voir les TODO.
"""

import numpy as np

import config
from robot import Robot
from Controllers.pid_controller import PID


class AutoTuner:
    def __init__(self, initial_state=None, target_state=None, sim_time=20.0):
        # Paramètres à régler : [Kp, Ki, Kd, Pos_Kp, Pos_Kd]
        self.params = [0.0, 0.0, 0.0, 0.0, 0.0]

        # TODO 1 : choisir les pas d'exploration initiaux, un par paramètre.
        #   Ils doivent être à l'échelle de chaque gain : la commande est un
        #   rapport cyclique dans [-1, 1], donc le Kp d'angle se compte en
        #   dizaines alors que les gains de position valent quelques centièmes.
        #   Un pas nul revient à NE PAS régler le paramètre (c'est le choix fait
        #   ici pour Ki : l'intégrateur est laissé à zéro).
        self.dparams = None

        if initial_state is None:
            initial_state = np.array([0.1, 0.0, -0.2, 0.0])
        if target_state is None:
            target_state = np.zeros(4)

        self.initial_state = np.array(initial_state, dtype=float)
        self.target_state = np.array(target_state, dtype=float)
        self.sim_time = sim_time
        self.u_max = config.PWM_MAX

        # États initiaux supplémentaires pour l'évaluation robuste.
        self.etats_robustesse = [
            np.array([-0.3, 0.0, 0.15, 0.0]),
            np.array([0.0, 0.0, 0.05, 0.0]),
        ]

        self.best_cost = np.inf
        self.eval_count = 0

    def _simulate(self, params, initial_state=None):
        """
        Simule la cascade complète avec un jeu de gains et mesure sa qualité.

        Args:
            params: [Kp, Ki, Kd, Pos_Kp, Pos_Kd]
            initial_state: état de départ (celui du tuner par défaut)

        Returns:
            dict de métriques, ou {'crash_time': t} si le robot est tombé.
        """
        # TODO 2 : interdire les gains négatifs. Ils n'ont aucun sens physique
        #          (ils poussent le robot DANS le sens de la chute) et
        #          l'algorithme s'y engouffre volontiers.

        # TODO 3 : créer les deux PID et le Robot, puis dérouler la boucle de
        #          simulation avec EXACTEMENT le même câblage que
        #          Controllers/main_controller.py. Si les deux diffèrent, on
        #          optimise un système que l'on ne simulera jamais.

        # TODO 4 : détecter la chute (|theta| > pi/2) et renvoyer
        #          {'crash_time': i * config.dt}

        # TODO 5 : accumuler et renvoyer les métriques :
        #   'total_angle_error' : intégrale de l'erreur d'angle au carré
        #   'total_pos_error'   : intégrale de l'erreur de position au carré
        #   'total_effort'      : intégrale de u^2 (consommation)
        #   'settling_time_theta' / 'settling_time_x' : dernier instant où
        #       l'erreur dépasse la tolérance (0.01 rad ; 0.05 m)
        #   'max_overshoot_theta' / 'max_overshoot_x' : dépassements maximaux
        #   'osc_theta' / 'osc_x' : ÉCART-TYPE sur le dernier quart de la
        #       simulation. Pourquoi l'écart-type et pas l'amplitude
        #       crête-à-crête : cette dernière dépend d'un seul point, donc
        #       elle varie brutalement, et l'optimiseur exploite ces sauts pour
        #       trouver des optima "en lame de rasoir" — excellents en
        #       simulation, inutilisables sur un vrai robot.
        #   'mean_x_tail' : dérive résiduelle de position en fin de simulation
        raise NotImplementedError("AutoTuner._simulate : à implémenter")

    def _cout_trajectoire(self, result):
        """
        Transforme les métriques en un seul nombre : le coût.

        Les pondérations traduisent nos priorités :

            poids_1   * settling_time_theta      poids_2   * settling_time_x
            poids_3   * total_angle_error        poids_4   * total_pos_error
            poids_5   * total_effort             poids_6   * max_overshoot_theta
            poids_7   * osc_theta                poids_8   * osc_x
            poids_9   * mean_x_tail

        Cas de la CHUTE : ne surtout pas renvoyer une constante. L'erreur
        cumulée grandit avec le temps, donc une pénalité fixe apprendrait à
        l'optimiseur qu'il vaut mieux tomber TÔT. Il faut une pénalité GRADUÉE,
        qui diminue avec le temps de survie, par exemple :

            1e5 + 1e5 * (1 - crash_time / sim_time)
        """
        # TODO 6 : implémenter les deux cas (chute / trajectoire complète)
        raise NotImplementedError("AutoTuner._cout_trajectoire : à implémenter")

    def run_simulation(self, params):
        """
        Coût d'un jeu de gains : le PIRE cas sur tous les états initiaux.

        Retenir le pire cas plutôt que la moyenne évite de sélectionner des
        gains excellents sur une trajectoire et catastrophiques sur une autre.
        """
        # TODO 7 : incrémenter self.eval_count, évaluer sur
        #          [initial_state] + etats_robustesse et renvoyer le max
        raise NotImplementedError("AutoTuner.run_simulation : à implémenter")

    def twiddle(self, tol=0.05, verbose=True):
        """
        Boucle principale de l'algorithme.

        Returns:
            list: les cinq gains optimisés
        """
        # TODO 8 : implémenter twiddle. Ne pas oublier de sauter les paramètres
        #          dont le pas est nul, et de re-saturer les gains à zéro quand
        #          la descente les rend négatifs.
        raise NotImplementedError("AutoTuner.twiddle : à implémenter")


def update_config_file(kp, ki, kd, pos_kp=None, pos_kd=None, config_path="config.py"):
    """
    Réécrit les gains PID dans config.py.

    """
    print(">>> Sauvegarde des nouveaux gains dans config.py...")

    remplacements = {
        "PID_Kp": f"{kp:.6f}",
        "PID_Ki": f"{ki:.6f}",
        "PID_Kd": f"{kd:.6f}",
    }
    if pos_kp is not None:
        remplacements["PID_Pos_Kp"] = f"{pos_kp:.6f}"
    if pos_kd is not None:
        remplacements["PID_Pos_Kd"] = f"{pos_kd:.6f}"

    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    with open(config_path, "w", encoding="utf-8") as f:
        for line in lines:
            nom = line.split("=")[0].strip()
            if nom in remplacements:
                commentaire = ""
                if "#" in line:
                    commentaire = "  " + line[line.index("#"):].rstrip()
                f.write(f"{nom} = {remplacements[nom]}{commentaire}\n")
            else:
                f.write(line)


def run_optimization(initial_state=None, target_state=None, sim_time=20.0,
                     save_to_config=True):
    """
    Fonction appelée par MainController.

    Returns:
        list: [Kp, Ki, Kd, Pos_Kp, Pos_Kd]
    """
    # TODO 9 : créer l'AutoTuner, lancer twiddle(), écrire le résultat dans
    #          config.py si save_to_config, et renvoyer les gains
    raise NotImplementedError("pid_optimizer.run_optimization : à implémenter")


if __name__ == "__main__":
    run_optimization()
