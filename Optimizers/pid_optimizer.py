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
import os
import shutil
import numpy as np

import config
from robot import Robot
from Controllers.pid_controller import PIDCascade


NOMS_GAINS = ["Kp", "Ki", "Kd", "Pos_Kp", "Pos_Ki", "Pos_Kd"]

# Ordre de grandeur de référence de chaque gain. Il sert à NORMALISER les pas
# d'exploration : sans lui, twiddle compare un pas de 10 sur Kp à un pas de
# 0.02 sur Pos_Kd, le critère d'arrêt est dominé par le premier, et les gains
# de position voient leur pas s'effondrer avant d'avoir été explorés.
ECHELLES = np.array([20.0, 5.0, 0.5, 0.05, 0.01, 0.05])


class AutoTuner:
    """
    Auto-tuning de la cascade PID par la méthode "twiddle" (recherche par
    coordonnées à pas adaptatif).

    Les SIX gains de la cascade sont réglés ensemble :
        [Kp, Ki, Kd, Pos_Kp, Pos_Ki, Pos_Kd]

    Points importants du réglage, et pourquoi :

    * MEME loi de commande que la simulation. Le contrôleur est la classe
      PIDCascade, celle qu'utilise main_controller. Auparavant la loi était
      recopiée ici, et main_controller débranchait la boucle de position
      pendant l'auto-tuning : les gains étaient optimisés pour un système et
      évalués sur un autre.

    * Divergence en position = échec. Un robot qui part à 10 m en restant
      debout n'est pas une solution ; sans la borne config.CHUTE_X_MAX,
      l'optimiseur en accepte.

    * Pas NORMALISÉS (voir ECHELLES) et départ depuis les gains de config.py.
      Partir de zéro obligeait la recherche à d'abord « survivre », phase
      pendant laquelle tout coûte 1e5 et où seuls les gains d'angle comptent.

    * Recherche sur un horizon court (search_time), validation finale sur
      l'horizon complet. La dynamique est établie bien avant 20 s et chaque
      évaluation coûte trois simulations.

    La commande est un RAPPORT CYCLIQUE dans [-1, 1] : les gains nécessaires
    sont bien plus grands qu'à l'époque du couple.
    """

    def __init__(self, initial_state=None, target_state=None, sim_time=20.0,
                 search_time=8.0):
        if initial_state is None:
            initial_state = np.array([0.1, 0.0, -0.2, 0.0])
        if target_state is None:
            target_state = np.zeros(4)

        self.initial_state = np.array(initial_state, dtype=float)
        self.target_state = np.array(target_state, dtype=float)
        self.sim_time = sim_time
        # Horizon utilisé PENDANT la recherche (jamais plus long que la
        # simulation finale).
        self.search_time = min(search_time, sim_time)
        self.u_max = config.PWM_MAX

        # Point de départ : les gains déjà en place. Ils sont en général déjà
        # viables, la recherche n'a donc plus à trouver la stabilité d'abord.
        self.params = [config.PID_Kp, config.PID_Ki, config.PID_Kd,
                       config.PID_Pos_Kp, config.PID_Pos_Ki, config.PID_Pos_Kd]

        # Pas d'exploration, exprimés en fraction de ECHELLES (donc homogènes
        # entre eux). Tous les gains démarrent avec la même « ambition ».
        self.dparams = [0.30] * len(self.params)

        # États initiaux supplémentaires pour l'évaluation robuste : optimiser
        # sur une seule trajectoire revient à surapprendre ce cas précis.
        # Identiques à ceux du FuzzyAutoTuner pour que la comparaison entre
        # méthodes reste équitable.
        self.etats_robustesse = [
            np.array([-0.3, 0.0, 0.15, 0.0]),
            np.array([0.0, 0.0, 0.05, 0.0]),
        ]

        self.best_cost = np.inf
        self.eval_count = 0

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------
    def _simulate(self, params, initial_state=None, sim_time=None):
        """
        Simule la cascade PID complète.
        params = [Kp, Ki, Kd, Pos_Kp, Pos_Ki, Pos_Kd]

        Returns:
            dict de métriques, ou dict avec 'crash_time' si chute/divergence.
        """
        # Des gains négatifs n'ont aucun sens physique (ils poussent le robot
        # dans le sens de la chute) : on les interdit explicitement.
        kp, ki, kd, pos_kp, pos_ki, pos_kd = [max(0.0, v) for v in params]
        sim_time = self.sim_time if sim_time is None else sim_time

        cascade = PIDCascade(kp, ki, kd, pos_kp, pos_ki, pos_kd, config.dt,
                             u_max=self.u_max)

        bot = Robot()
        state = (self.initial_state if initial_state is None else initial_state).copy()
        steps = int(sim_time / config.dt)

        total_angle_error = 0.0
        total_pos_error = 0.0
        total_effort = 0.0
        max_overshoot_theta = 0.0
        max_overshoot_x = 0.0
        settling_time_theta = sim_time
        settling_time_x = sim_time

        theta_tol = 0.01   # 0.01 rad ~ 0.6 deg
        x_tol = 0.05       # 5 cm

        theta_traj = np.empty(steps)
        x_traj = np.empty(steps)

        for i in range(steps):
            u, _ = cascade.compute(state, target_x=self.target_state[0])
            state = bot.step(state, u, config.dt)
            theta_traj[i] = state[2]
            x_traj[i] = state[0]

            # Échec : pénalité GRADUÉE selon le temps de survie (voir _cout).
            # La divergence en position compte comme un échec au même titre que
            # la chute : sur le robot réel, partir à 1.5 m de la consigne n'est
            # pas un asservissement réussi — et de toute façon, à cette allure
            # la fcem finit par supprimer le couple disponible.
            if abs(state[2]) > config.CHUTE_THETA_MAX or abs(state[0]) > config.CHUTE_X_MAX:
                return {'crash_time': i * config.dt, 'sim_time': sim_time}

            t = i * config.dt
            err_theta = abs(state[2] - self.target_state[2])
            err_x = abs(state[0] - self.target_state[0])

            total_angle_error += err_theta ** 2
            total_pos_error += err_x ** 2
            total_effort += u ** 2

            max_overshoot_theta = max(max_overshoot_theta, err_theta)
            max_overshoot_x = max(max_overshoot_x, err_x)

            # Temps de stabilisation = dernier instant hors tolérance.
            if err_theta > theta_tol:
                settling_time_theta = t
            if err_x > x_tol:
                settling_time_x = t

        # Oscillation résiduelle sur le dernier quart, en écart-type (métrique
        # continue, contrairement au crête-à-crête qu'un seul point fait varier).
        tail = slice(int(0.75 * steps), steps)
        return {
            'sim_time': sim_time,
            'osc_theta': float(np.std(theta_traj[tail])),
            'osc_x': float(np.std(x_traj[tail])),
            'osc_theta_crete': float(theta_traj[tail].max() - theta_traj[tail].min()),
            'osc_x_crete': float(x_traj[tail].max() - x_traj[tail].min()),
            'mean_x_tail': float(np.abs(x_traj[tail].mean())),
            'total_angle_error': total_angle_error * config.dt,
            'total_pos_error': total_pos_error * config.dt,
            'total_effort': total_effort * config.dt,
            'settling_time_theta': settling_time_theta,
            'settling_time_x': settling_time_x,
            'max_overshoot_theta': max_overshoot_theta,
            'max_overshoot_x': max_overshoot_x,
            'max_x': float(np.abs(x_traj).max()),
        }

    # ------------------------------------------------------------------
    # Coût
    # ------------------------------------------------------------------
    def _cout_trajectoire(self, result):
        """
        Coût d'une trajectoire.

        Les poids des termes de POSITION ont été alignés sur ceux de l'angle.
        Avec l'ancienne pondération, faire varier Kp d'un facteur 2 changeait le
        coût d'un facteur 2 à 3, alors que faire varier Pos_Kp du même facteur
        ne le changeait que de 10 % : twiddle, qui rétrécit le pas dès qu'un
        essai échoue, abandonnait les directions de position avant de les avoir
        explorées. D'où l'impression que « l'optimiseur ne règle que l'angle ».
        """
        if 'crash_time' in result:
            # Pénalité graduée : tomber tard coûte moins cher que tomber tôt,
            # ce qui donne une direction de progrès depuis les zones instables.
            # Une pénalité fixe serait piégeuse, l'erreur cumulée augmentant
            # avec le temps : l'optimiseur apprendrait à faire tomber le robot
            # le plus vite possible.
            return 1e5 + 1e5 * (1.0 - result['crash_time'] / result['sim_time'])

        return (
            10.0 * result['settling_time_theta']
            + 10.0 * result['settling_time_x']
            + 50.0 * result['total_angle_error']
            + 50.0 * result['total_pos_error']
            + 0.01 * result['total_effort']
            + 20.0 * result['max_overshoot_theta']
            + 20.0 * result['max_overshoot_x']
            + 300.0 * result['osc_theta']
            + 300.0 * result['osc_x']
            + 300.0 * result['mean_x_tail']
        )

    def run_simulation(self, params, sim_time=None, cutoff=None):
        """
        Coût d'un jeu de gains : PIRE CAS sur plusieurs états initiaux.

        Retenir le pire cas plutôt que la moyenne évite de sélectionner des
        gains excellents sur une trajectoire et médiocres ailleurs.

        `cutoff` : le coût étant un MAX, dès qu'un état dépasse le meilleur coût
        connu, la suite ne peut plus faire baisser le résultat. On arrête donc
        l'évaluation là (sortie anticipée) : c'est ce qui rend le tuning
        utilisable en quelques minutes au lieu de plus d'un quart d'heure.
        """
        self.eval_count += 1
        sim_time = self.search_time if sim_time is None else sim_time
        etats = [self.initial_state] + self.etats_robustesse

        pire = -np.inf
        for etat in etats:
            cout = self._cout_trajectoire(self._simulate(params, etat, sim_time))
            pire = max(pire, cout)
            if cutoff is not None and pire >= cutoff:
                return pire   # inutile de simuler les états restants
        return pire

    # ------------------------------------------------------------------
    # Recherche
    # ------------------------------------------------------------------
    def twiddle(self, tol=0.02, verbose=True):
        """
        tol porte sur le pas NORMALISÉ le plus grand (fraction de ECHELLES),
        et non plus sur la somme de pas hétérogènes que dominait dparams[0].
        """
        if verbose:
            print(">>> Démarrage de l'auto-tuning de la cascade PID...")
            print(f"    Horizon de recherche : {self.search_time:.1f} s "
                  f"(validation finale à {self.sim_time:.1f} s)")
            print(f"    Départ : {self._format_gains(self.params)}")

        best_err = self.run_simulation(self.params)

        while max(self.dparams) > tol:
            for i in range(len(self.params)):
                pas = self.dparams[i] * ECHELLES[i]
                # On mémorise la valeur de départ : la restaurer par
                # "+= pas" après un "-= 2*pas" est faux dès que la borne à
                # zéro a joué (gains positifs uniquement).
                valeur_initiale = self.params[i]

                self.params[i] = valeur_initiale + pas
                err = self.run_simulation(self.params, cutoff=best_err)

                if err < best_err:
                    best_err = err
                    self.dparams[i] *= 1.1
                    continue

                # On reste dans le domaine physique (gains positifs)
                self.params[i] = max(0.0, valeur_initiale - pas)
                err = self.run_simulation(self.params, cutoff=best_err)

                if err < best_err:
                    best_err = err
                    self.dparams[i] *= 1.1
                else:
                    self.params[i] = valeur_initiale
                    self.dparams[i] *= 0.9

            if verbose:
                print(f"    ... coût={best_err:10.3f} | pas max={max(self.dparams):.4f} "
                      f"| {self.eval_count} éval.")

        self.params = [max(0.0, v) for v in self.params]

        # Validation sur l'horizon COMPLET : les gains sont cherchés vite, mais
        # jugés sur la durée qui sera réellement simulée.
        self.best_cost = self.run_simulation(self.params, sim_time=self.sim_time)

        if verbose:
            print(f">>> Tuning terminé ({self.eval_count} évaluations)")
            print(f"    Coût (horizon complet {self.sim_time:.0f} s) : {self.best_cost:.4f}")
            print(f"    {self._format_gains(self.params)}")
            self.rapport(self.params)

        return self.params

    def rapport(self, params):
        """Affiche les métriques du jeu de gains sur l'horizon complet."""
        m = self._simulate(params, sim_time=self.sim_time)
        if 'crash_time' in m:
            print(f"    !! ÉCHEC à t={m['crash_time']:.2f}s sur l'état nominal")
            return
        print(f"    Stabilisation θ/x    : {m['settling_time_theta']:.2f} s / {m['settling_time_x']:.2f} s")
        print(f"    Oscillation θ (crête): {np.degrees(m['osc_theta_crete']):.2f}°")
        print(f"    Excursion max en x   : {m['max_x']:.3f} m")
        print(f"    Dérive résiduelle x  : {m['mean_x_tail']:.4f} m")

    @staticmethod
    def _format_gains(params):
        return " ".join(f"{n}={v:.4f}" for n, v in zip(NOMS_GAINS, params))


# ----------------------------------------------------------------------
# Sauvegarde
# ----------------------------------------------------------------------
def update_config_file(kp, ki, kd, pos_kp=None, pos_ki=None, pos_kd=None,
                       config_path="config.py"):
    """
    Lit config.py, remplace les lignes PID_... et sauvegarde.

    Une copie de sauvegarde config.py.bak est écrite avant toute modification :
    un tuning raté écrasait auparavant les gains en place sans recours.
    """
    print(">>> Sauvegarde des nouveaux gains dans config.py...")

    if os.path.exists(config_path):
        shutil.copyfile(config_path, config_path + ".bak")
        print(f"    (sauvegarde de l'ancien fichier dans '{config_path}.bak')")

    remplacements = {
        "PID_Kp": f"{kp:.6f}",
        "PID_Ki": f"{ki:.6f}",
        "PID_Kd": f"{kd:.6f}",
    }
    if pos_kp is not None:
        remplacements["PID_Pos_Kp"] = f"{pos_kp:.6f}"
    if pos_ki is not None:
        remplacements["PID_Pos_Ki"] = f"{pos_ki:.6f}"
    if pos_kd is not None:
        remplacements["PID_Pos_Kd"] = f"{pos_kd:.6f}"

    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    with open(config_path, "w", encoding="utf-8") as f:
        for line in lines:
            nom = line.split("=")[0].strip()
            if nom in remplacements:
                # On conserve le commentaire de fin de ligne s'il y en a un
                commentaire = ""
                if "#" in line:
                    commentaire = "  " + line[line.index("#"):].rstrip()
                f.write(f"{nom} = {remplacements[nom]}{commentaire}\n")
            else:
                f.write(line)  # On réécrit les autres lignes à l'identique


def run_optimization(initial_state=None, target_state=None, sim_time=20.0,
                     search_time=8.0, save_to_config=False):
    """
    Fonction principale appelée par main.py.

    `save_to_config` est FALSE par défaut : importer ce module et lancer une
    optimisation ne doit pas réécrire config.py à l'insu de l'utilisateur. Même
    quand il est activé, les gains ne sont écrits que s'ils font MIEUX que ceux
    déjà en place, évalués avec exactement le même coût.
    """
    tuner = AutoTuner(initial_state=initial_state, target_state=target_state,
                      sim_time=sim_time, search_time=search_time)

    # Référence : les gains actuellement dans config.py, sur l'horizon complet.
    gains_actuels = list(tuner.params)
    cout_actuel = tuner.run_simulation(gains_actuels, sim_time=sim_time)
    print(f">>> Coût des gains actuels de config.py : {cout_actuel:.4f}")

    best_params = tuner.twiddle()

    if save_to_config:
        if tuner.best_cost < cout_actuel:
            print(f">>> Amélioration : {cout_actuel:.4f} -> {tuner.best_cost:.4f}")
            update_config_file(*best_params)
        else:
            print(f">>> Aucune amélioration ({tuner.best_cost:.4f} >= {cout_actuel:.4f}) : "
                  f"config.py est laissé intact, on garde les gains existants.")
            best_params = gains_actuels

    return best_params  # On retourne les valeurs pour usage immédiat


if __name__ == "__main__":
    run_optimization(save_to_config=True)

