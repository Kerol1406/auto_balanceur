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

        # UNE SEULE boucle : à chaque pas, on calcule la commande, on
        # avance la simulation, PUIS on accumule les métriques sur ce
        # même pas. (Le bug précédent avait une 2e boucle imbriquée qui
        # exécutait toute la simulation avant de sortir, faussant les
        # métriques et multipliant le coût de calcul par `steps`.)
        for i in range(steps):
            u = lqr.compute(state, target_state=self.target_state)
            u = np.clip(u, -self.u_max, self.u_max)
            state = robot.step(state, u, config.dt)

            if abs(state[2]) > 40 * np.pi / 180:
                return None

            t = i * config.dt
            err_theta = abs(state[2] - self.target_state[2])
            err_x = abs(state[0] - self.target_state[0])

            total_angle_error += err_theta**2
            total_pos_error += err_x**2
            total_effort += u**2
            max_overshoot_theta = max(max_overshoot_theta, state[2])
            max_overshoot_x = max(max_overshoot_x, state[0])

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

        Q_diag = params[:-1]
        R_val = params[-1]

        result = self._simulate(Q_diag, R_val)

        if result is None:
            return 1e8

        W = np.array([35, 30, 50, 40, 23, 19]).T
        X = np.array([result["settling_time_theta"], result["settling_time_x"], result["total_angle_error"],
                      result["total_pos_error"], result["total_effort"], result["max_overshoot_theta"]])
        cost = W @ X

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
            bounds = [
                (0, 200),      # q_x
                (0, 100),      # q_xdot
                (0, 500),      # q_theta
                (0, 50),       # q_thetadot
                (0.01, 10),    # R
            ]

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

        result = differential_evolution(self._cost_function,
                                         bounds=bounds,
                                         maxiter=maxiter,
                                         seed=seed, tol=1e-5, polish=True)

        print(f"Temps mis : {time.time() - debut}")

        Q_opt = np.diag(result.x[:4])
        R_opt = result.x[4]

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