import numpy as np
import config

from Controllers.lqr_controller import LQR
from robot import Robot
from scipy.optimize import differential_evolution
import time


class LQRAutoTuner:
    """
    Auto-tuning des matrices Q et R du LQR par differential_evolution
    (algorithme génétique / évolution différentielle).

    Idée générale : au lieu de choisir Q et R à la main puis d'observer le
    comportement du robot, on laisse un algorithme d'optimisation explorer
    l'espace des Q/R possibles, en jugeant chaque candidat par une SIMULATION
    complète du robot (pas de gradient, pas de formule fermée -- juste
    "est-ce que ce jeu de Q/R donne un bon comportement en pratique ?").

    Optimise les 4 diagonales de Q + R en simulant le robot et en minimisant
    une fonction de coût basée sur le temps de convergence, l'overshoot et
    l'effort moteur.

    Prérequis : la classe LQR (controller/lqr_controller.py) doit déjà être
    complète et fonctionnelle -- ce fichier l'utilise telle quelle, sans la
    modifier.
    """

    def __init__(self, initial_state, target_state, sim_time=20.0, u_max=1.0):
        """
        Args:
            initial_state: État initial [x, x_dot, theta, theta_dot]
            target_state: État cible [x, x_dot, theta, theta_dot]
            sim_time: Durée de simulation (s)
            u_max: Saturation moteur (rapport cyclique max, dans [-1, 1] typiquement)
        """
        self.initial_state = np.array(initial_state)
        self.target_state = np.array(target_state)
        self.sim_time = sim_time
        self.u_max = u_max
        self.best_cost = np.inf
        self.eval_count = 0
        print('state optim : ', self.initial_state)

    def _simulate(self, Q_diag, R_val):
        """
        Simule le robot avec un LQR paramétré par Q et R.

        Returns:
            dict avec les métriques de performance, ou None si crash/instabilité.
        """
        try:
            Q = np.diag(Q_diag) if np.ndim(Q_diag) == 1 else np.array(Q_diag)
            lqr = LQR(Q=Q, R=R_val, verbose=False)
        except Exception:
            return None

        stability = lqr.check_stability()
        if not stability['stable']:
            return None

        robot = Robot()
        state = self.initial_state.copy()
        steps = int(self.sim_time / config.dt)

        total_angle_error = 0.0
        total_pos_error = 0.0
        total_effort = 0.0
        max_overshoot_theta = 0.0
        max_overshoot_x = 0.0
        settling_time_theta = self.sim_time
        settling_time_x = self.sim_time
        theta_settled = False
        x_settled = False

        theta_tol = 0.01   # 0.01 rad ≈ 0.6°
        x_tol = 0.02       # 5 cm

        for i in range(steps):
            # ============================================================
            # TODO 1 : Boucle de contrôle -- appliquer le LQR et faire
            # avancer la simulation physique
            # ============================================================
            # C'est exactement la même logique que dans un vrai firmware
            # (lire l'état, calculer la commande, l'appliquer, mesurer le
            # nouvel état), mais ici le "robot" est simulé pas à pas.
            #
            # Étapes à coder, dans l'ordre :
            #   1. Calculer la commande LQR avec lqr.compute(état_actuel,
            #      target_state=self.target_state) -- 
            #      cette méthode a été déjà implémenté dans l'exercice précédent (TODO 3 du LQR).
            #   2. Saturer cette commande entre -self.u_max et self.u_max
            #      avec np.clip(valeur, min, max).
            #   3. Fais avancer la simulation d'un pas de temps avec
            #      bot.step(état_actuel, commande, config.dt), qui renvoie
            #      le nouvel état -- écrase la variable `state` avec ce
            #      résultat.
            #   4. Détecter un "crash" : si l'angle (état d'indice 2) dépasse
            #      40° en valeur absolue (converti en radians), la
            #      simulation n'a plus de sens -- renvoie None immédiatement
            #      pour arrêter cette simulation ici.
            #
            # Indice : 40° en radians s'écrit `40 * np.pi / 180`.
            for i in range(steps):
                u = lqr.compute(state,target_state=self.target_state)      # <-- à remplacer (TODO 1, étape 1)
                u = np.clip(u, -self.u_max,self.u_max)     # <-- à remplacer (TODO 1, étape 2 : clip)
                state = robot.step(state, u, config.dt)  # <-- à remplacer (TODO 1, étape 3)

            # DONE 1, étape 4 : détection de crash (if ... : return None)
            if abs(state[2]) > 70 * np.pi / 180:
                return None
            # ============================================================
            # FIN TODO 1
            # ============================================================

            t = i * config.dt
            err_theta = abs(state[2] - self.target_state[2])
            err_x = abs(state[0] - self.target_state[0])

            # ============================================================
            # DONE 2 : Accumulation des métriques de performance
            # ============================================================
            # Objectif : construire, au fil de la simulation, des
            # indicateurs résumant "à quel point ce jeu de Q/R est bon".
            #
            # 4 quantités à mettre à jour à chaque pas :
            #
            #   a. total_angle_error : somme cumulée de err_theta au carré
            #      (erreur QUADRATIQUE, comme dans un coût x'Qx -- ça pénalise
            #      plus fortement les grosses erreurs que les petites)
            #
            #   b. total_pos_error : même principe, mais avec err_x au carré
            #
            #   c. total_effort : somme cumulée de la commande u au carré
            #      (représente l'énergie dépensée par le moteur, comme le
            #      terme u'Ru du LQR)
            #
            #   d. max_overshoot_theta et max_overshoot_x : le PLUS GRAND
            #      écart observé depuis le début de la simulation (pas une
            #      somme -- un maximum). Indice : max(valeur_actuelle,
            #      nouvelle_mesure)
            #
            # Pense à utiliser des += pour les sommes (a, b, c), et à
            # réassigner max_overshoot_theta / max_overshoot_x avec max(...)
            # pour le maximum (d).

            total_angle_error += err_theta**2   # <-- à compléter
            total_pos_error += err_x**2     # <-- à compléter
            total_effort += u**2        # <-- à compléter
            max_overshoot_theta = max(max_overshoot_theta,state[2])  # <-- à compléter
            max_overshoot_x = max(max_overshoot_x,state[0])      # <-- à compléter

            # ============================================================
            # FIN TODO 2
            # ============================================================

            # ============================================================
            # DONE 3 : Suivi du temps de stabilisation (settling time)
            # ============================================================
            # Objectif : déterminer À QUEL INSTANT le système est rentré
            # DÉFINITIVEMENT dans la zone de tolérance (theta_tol, x_tol) et
            # n'en est plus jamais ressorti.
            #
            # C'est une petite "machine à états" : à chaque pas de temps, si
            # l'erreur dépasse encore la tolérance, on repousse le temps de
            # stabilisation à l'instant présent (car le système n'est "pas
            # encore stable" tant qu'il oscille au-delà de la tolérance).
            #
            # Logique à coder pour theta (fais EXACTEMENT la même chose pour
            # x ensuite, avec x_tol, settling_time_x, x_settled) :
            #
            #   si err_theta > theta_tol:
            #       # on est encore hors tolérance : on repousse le temps
            #       # de stabilisation à maintenant, et on note qu'on n'est
            #       # plus "stabilisé"
            #       settling_time_theta = t
            #       theta_settled = False
            #   sinon, si theta_settled est actuellement False:
            #       # on vient tout juste de rentrer dans la tolérance :
            #       # on le note (mais on NE modifie PAS settling_time_theta
            #       # ici -- il reste à sa dernière valeur "hors tolérance")
            #       theta_settled = True
            #
            # Indice : structure en if / elif, comme décrit ci-dessus.
            # Recopie la même logique juste en dessous pour x (avec les
            # variables x_tol, settling_time_x, x_settled, err_x).

            if err_theta > theta_tol:
                settling_time_theta = t
                theta_settled = False
            elif not theta_settled:
                theta_settled = True

            if err_x > x_tol:
                settling_time_x = t
                x_settled = False
            elif not x_settled:
                x_settled = True

            # ============================================================
            # FIN TODO 3
            # ============================================================

        return {
            'total_angle_error': total_angle_error * config.dt,
            'total_pos_error': total_pos_error * config.dt,
            'total_effort': total_effort * config.dt,
            'settling_time_theta': settling_time_theta,
            'settling_time_x': settling_time_x,
            'max_overshoot_theta': max_overshoot_theta,
            'max_overshoot_x': max_overshoot_x,
        }

    def _cost_function(self, params):
        """
        Fonction de coût pour differential_evolution.
        params = [q_x, q_xdot, q_theta, q_thetadot, R]
        """
        self.eval_count += 1

        # ============================================================
        # DONE 4 : Fonction de coût pondérée
        # ============================================================
        # `params` est un vecteur de 5 nombres, dans l'ordre :
        #   [q_x, q_xdot, q_theta, q_thetadot, R]
        # (c'est differential_evolution qui choisit ces 5 valeurs, en
        # respectant les bornes qu'on lui donnera au TODO 5).
        #
        # Étape a : extrais Q_diag (les 4 premières valeurs de params) et
        # R_val (la 5ème), puis simule avec self._simulate(Q_diag, R_val).
        #
        # Étape b : si result est None (crash ou instabilité détectés dans
        # _simulate), renvoie une pénalité énorme -- 1e8 -- pour signaler à
        # l'optimiseur que ce candidat est très mauvais, sans faire planter
        # le programme.
        #
        # Étape c : sinon, combine les métriques en un seul scalaire, en
        # pondérant chaque terme selon son importance relative (poids déjà
        # choisis ci-dessous, à toi de les combiner en une formule) :
        #
        #   - w1 x settling_time_theta   (convergence rapide de l'angle)
        #   - w2  x settling_time_x       (convergence rapide de la position)
        #   - w3 x total_angle_error     (erreur angulaire cumulée)
        #   - w4 x total_pos_error       (erreur de position cumulée)
        #   - w5 x total_effort          (économie d'énergie, poids faible)
        #   - w6 x max_overshoot_theta   (pénaliser les dépassements d'angle) avec w_i >= 0
        #
        # Additionne ces 6 termes pour obtenir `cost`.

        Q_diag = params[:-1]  # <-- à remplacer (TODO 4a)
        R_val = params[-1]   # <-- à remplacer (TODO 4a)

        result = self._simulate(Q_diag, R_val)  # <-- à remplacer (TODO 4a : appel à self._simulate)

        # DONE 4b : if result is None: return 1e8
        if result is None:
            return 1e8
        W = np.array([35, 30, 50, 40, 23, 19]).T
        X = np.array([result["settling_time_theta"], result["settling_time_x"], result["total_angle_error"],
                      result["total_pos_error"], result["total_effort"], result["max_overshoot_theta"]])
        cost = W@X

        # <-- à remplacer (TODO 4c : combinaison pondérée)

        # ============================================================
        # FIN TODO 4
        # ============================================================

        if cost < self.best_cost:
            self.best_cost = cost
            print(f"  [#{self.eval_count}] Nouveau meilleur coût: {cost:.2f} | "
                  f"Q=diag({np.array(Q_diag).round(2)}) R={R_val:.3f} | "
                  f"t_theta={result['settling_time_theta']:.2f}s t_x={result['settling_time_x']:.2f}s")

        return cost

    def optimize(self, bounds=None, maxiter=2, seed=42):
        """
        Lance l'optimisation par differential_evolution.

        Args:
            bounds: Bornes [(min, max)] pour [q_x, q_xdot, q_theta, q_thetadot, R]
            maxiter: Nombre max de générations
            seed: Graine aléatoire pour reproductibilité

        Returns:
            dict: Résultat avec Q, R optimaux et les métriques
        """
        if bounds is None:
            # ============================================================
            # DONE 5 : Bornes de recherche pour l'algorithme génétique
            # ============================================================
            # Objectif : définir, pour chacun des 5 paramètres, un
            # intervalle [min, max] raisonnable dans lequel l'algorithme va
            # chercher. Des bornes trop larges ralentissent la recherche
            # (espace à explorer inutilement grand) ; des bornes trop
            # étroites peuvent exclure la vraie solution optimale.
            #
            # Rappel (voir la docstring de la classe LQR pour l'intuition
            # physique de Q et R) :
            #   - q_theta doit pouvoir devenir grand (c'est la priorité :
            #     stabiliser l'angle), par exemple jusqu'à plusieurs
            #     centaines.
            #   - q_x, q_xdot, q_thetadot peuvent rester dans des ordres de
            #     grandeur plus modestes (quelques dizaines à une centaine).
            #   - R doit rester positif et non nul (jamais 0 -- pense à
            #     pourquoi, en lien avec l'inversion de R dans le calcul de
            #     K), typiquement entre 0.01 et 10.
            #
            # Complète la liste ci-dessous avec des tuples (min, max), un
            # par paramètre, dans l'ordre [q_x, q_xdot, q_theta, q_thetadot, R] :

            bounds = [
                (0, 200),    # q_x       <-- à remplacer
                (0, 100),    # q_xdot    <-- à remplacer
                (0, 500),    # q_theta   <-- à remplacer
                (0, 50),    # q_thetadot <-- à remplacer
                (0.01, 10),    # R         <-- à remplacer
            ]
            # ============================================================
            # FIN TODO 5
            # ============================================================

        print("=" * 60)
        print("LQR AUTO-TUNER (differential_evolution)")
        print("=" * 60)
        print(f"État initial : {self.initial_state}")
        print(f"État cible   : {self.target_state}")
        print(f"Bornes       : {bounds}")
        print(f"Max itérations: {maxiter}")
        print("-" * 60)

        self.best_cost = np.inf
        self.eval_count = 0

        debut = time.time()

        # ============================================================
        # DONE 6 : Appel à differential_evolution
        # ============================================================
        # C'est ici qu'on lance réellement l'algorithme d'optimisation.
        # `differential_evolution` a besoin, au minimum, de deux arguments :
        #   - la fonction de coût à minimiser (celle que vous avez complétée
        #     au TODO 4)
        #   - les bornes de recherche (celles du TODO 5, ou passées en
        #     argument à optimize())
        #
        # En plus de ces deux arguments obligatoires, passe aussi :
        #   - maxiter=maxiter, seed=seed (déjà reçus en paramètres de
        #     cette méthode)
        #   - tol=1e-5 (tolérance de convergence : arrête si les progrès
        #     deviennent négligeables)
        #   - polish=True (raffine la meilleure solution trouvée avec une
        #     optimisation locale à la fin)
        #
        # Indice : `differential_evolution(fonction_de_cout, bounds=..., ...)`
        # -- le nom de la fonction de coût à passer est une méthode de
        # cette classe (self._cost_function), sans l'appeler (pas de
        # parenthèses -- on passe la fonction elle-même, scipy l'appellera
        # lui-même autant de fois que nécessaire). (Vous pouvez implémenter d'autres manière de faire pour 
        # l'algorithme génétiques)

        result =  differential_evolution(self._cost_function, 
                                         bounds=bounds, 
                                         maxiter=maxiter, 
                                         seed=seed, tol=1e-5, polish=True) # <-- à remplacer

        # ============================================================
        # FIN TODO 6
        # ============================================================

        print(f"Temps mis : {time.time() - debut}")

        # ============================================================
        # DONE 7 : Extraction du résultat final
        # ============================================================
        # `result.x` est le meilleur vecteur de 5 paramètres trouvé par
        # l'optimiseur, dans le même ordre que dans _cost_function :
        # [q_x, q_xdot, q_theta, q_thetadot, R].
        #
        # Reconstruis :
        #   - Q_opt : la matrice diagonale 4x4 à partir des 4 premières
        #     valeurs de result.x (indice : np.diag(...))
        #   - R_opt : la 5ème valeur de result.x (un simple float)

        Q_opt = np.diag(result.x[:4])  # <-- à remplacer
        R_opt = result.x[4]  # <-- à remplacer

        # ============================================================
        # FIN TODO 7
        # ============================================================

        # Simuler une dernière fois pour les métriques finales
        metrics = self._simulate(result.x[:4], R_opt)

        print("\n" + "=" * 60)
        print("RÉSULTAT OPTIMAL")
        print("=" * 60)
        print(f"Q = diag({result.x[:4].round(4)})")
        print(f"R = {R_opt:.4f}")
        print(f"Coût final : {result.fun:.4f}")
        print(f"Évaluations: {self.eval_count}")
        if metrics:
            print(f"Temps stabilisation theta : {metrics['settling_time_theta']:.2f} s")
            print(f"Temps stabilisation x : {metrics['settling_time_x']:.2f} s")
            print(f"Overshoot max theta    : {np.degrees(metrics['max_overshoot_theta']):.1f}°")
            print(f"Overshoot max x       : {metrics['max_overshoot_x']:.3f} m")
        print("=" * 60)

        return {
            'Q': Q_opt,
            'R': R_opt,
            'Q_diag': result.x[:4],
            'cost': result.fun,
            'metrics': metrics,
        }


if __name__ == "__main__":
    initial_state = np.array([0, 0.0, -50 * np.pi / 180, 0.0])
    target_state = np.array([0.0, 0.0, 0.0, 0.0])

    tuner = LQRAutoTuner(initial_state, target_state, sim_time=20.0)
    result = tuner.optimize(maxiter=5)
    LQR_Q = result['Q']
    LQR_R = result['R']