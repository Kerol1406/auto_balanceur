"""
Réglage automatique de la cascade floue.

CE QUE L'ON OPTIMISE (ET CE QUE L'ON NE TOUCHE PAS)
---------------------------------------------------
La table de règles et les fonctions d'appartenance restent FIXES : c'est
l'expertise du pilote, elle ne s'optimise pas. On ne règle que les paramètres
continus des deux boucles, soit huit nombres :

    boucle interne (angle -> commande PWM)
        k_angle, k_vitesse : gains d'entrée (dilatent l'univers de discours)
        c1, c2             : centres de sortie, pris symétriques
                             [-c2, -c1, 0, c1, c2]

    boucle externe (position -> angle cible)
        pk_x, pk_dx        : gains d'entrée sur [x, dx]
        pc1, pc2           : centres de sortie, en radians

Comme pour le LQR, on utilise `differential_evolution` : la fonction de coût
n'est pas dérivable et l'espace de recherche est vaste.

CONTRAINTE À FAIRE RESPECTER
----------------------------
Les centres doivent rester ORDONNÉS (c2 > c1, pc2 > pc1). Sinon "aller vite"
devient plus doux qu'"aller doucement" : le contrôleur reste cohérent
mathématiquement, mais on ne peut plus interpréter le résultat, ce qui fait
perdre tout l'intérêt de la logique floue. La façon la plus simple de le faire
respecter est de renvoyer un coût très élevé quand la contrainte est violée.

À COMPLÉTER : voir les TODO.
"""

import numpy as np
from scipy.optimize import differential_evolution

import config
from robot import Robot
from Controllers.fuzzy_controller import FuzzyController


class FuzzyAutoTuner:
    def __init__(self, initial_state, target_state, sim_time=20.0, u_max=None):
        """
        Args:
            initial_state: état initial [x, dx, theta, dtheta]
            target_state: état visé
            sim_time: durée de chaque simulation d'essai (s)
            u_max: saturation du rapport cyclique (config.PWM_MAX par défaut)
        """
        self.initial_state = np.array(initial_state)
        self.target_state = np.array(target_state)
        self.sim_time = sim_time
        self.u_max = config.PWM_MAX if u_max is None else u_max

        # Évaluation robuste : voir l'explication dans pid_optimizer.py
        self.etats_robustesse = [
            np.array([-0.3, 0.0, 0.15, 0.0]),
            np.array([0.0, 0.0, 0.05, 0.0]),
        ]

        self.best_cost = np.inf
        self.eval_count = 0

    def _simulate(self, params, initial_state=None):
        """
        Simule la cascade floue paramétrée.

        Args:
            params: [k_angle, k_vitesse, c1, c2, pk_x, pk_dx, pc1, pc2]
            initial_state: état de départ (celui du tuner par défaut)

        Returns:
            dict de métriques, ou {'crash_time': t} si chute.
        """
        # TODO 1 : construire les DEUX FuzzyController à partir des paramètres
        #          (centres symétriques [-c2, -c1, 0, c1, c2])

        # TODO 2 : dérouler la simulation avec le même câblage que
        #          Controllers/main_controller.py — conversion en degrés,
        #          signes inversés sur les vitesses, saturation de l'angle
        #          cible à ±config.FUZZY_TARGET_THETA_MAX

        # TODO 3 : renvoyer les mêmes métriques que pid_optimizer._simulate
        raise NotImplementedError("FuzzyAutoTuner._simulate : à implémenter")

    def _cost_function(self, params):
        """
        Fonction de coût passée à differential_evolution.

        Args:
            params: [k_angle, k_vitesse, c1, c2, pk_x, pk_dx, pc1, pc2]
        """
        # TODO 4 : rejeter les jeux de paramètres dont les centres ne sont pas
        #          ordonnés (voir l'en-tête)

        # TODO 5 : évaluer sur tous les états initiaux, renvoyer immédiatement
        #          une pénalité graduée en cas de chute, sinon retenir le PIRE
        #          coût (mêmes pondérations que les autres optimiseurs)

        # TODO 6 : afficher les nouveaux meilleurs coûts au fil de la recherche
        raise NotImplementedError("FuzzyAutoTuner._cost_function : à implémenter")

    def optimize(self, bounds=None, maxiter=15, seed=42):
        """
        Lance l'optimisation.

        Bornes conseillées pour démarrer :
            k_angle, k_vitesse : 0.2 à 6
            c1 : 0.005 à 0.5   |  c2 : 0.02 à 1.0     (rapports cycliques)
            pk_x : 0.5 à 40    |  pk_dx : 0.2 à 20
            pc1 : 0.005 à 0.15 |  pc2 : 0.02 à 0.30   (angles cibles, rad)

        Returns:
            dict: {'output_centers', 'input_gains', 'pos_output_centers',
                   'pos_input_gains', 'params', 'cost', 'metrics'}
                  (format utilisé par MainController et update_config_file)
        """
        # TODO 7 : appeler differential_evolution, reconstruire les quatre
        #          tableaux de paramètres à partir de result.x, resimuler pour
        #          les métriques finales et renvoyer le dictionnaire attendu
        raise NotImplementedError("FuzzyAutoTuner.optimize : à implémenter")


def update_config_file(result, config_path="config.py"):
    """
    Réécrit les paramètres flous dans config.py (fourni).

    Six décimales, et pas quatre : l'optimum trouvé peut être étroit, au point
    qu'un arrondi à 1e-4 suffit à dégrader visiblement le comportement.
    """
    print(">>> Sauvegarde des paramètres flous optimaux dans config.py...")

    def fmt(array):
        return "np.array([" + ", ".join(f"{v:.6f}" for v in array) + "])"

    remplacements = {
        "FUZZY_OUTPUT_CENTERS": fmt(result['output_centers']),
        "FUZZY_INPUT_GAINS": fmt(result['input_gains']),
        "FUZZY_POS_OUTPUT_CENTERS": fmt(result['pos_output_centers']),
        "FUZZY_POS_INPUT_GAINS": fmt(result['pos_input_gains']),
    }

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
                     maxiter=15, save_to_config=True):
    """Optimise la cascade floue et met à jour config.py."""
    # TODO 8 : valeurs par défaut des états, création du tuner, optimisation,
    #          sauvegarde, puis renvoi du résultat
    raise NotImplementedError("fuzzy_optimizer.run_optimization : à implémenter")


if __name__ == "__main__":
    run_optimization()
