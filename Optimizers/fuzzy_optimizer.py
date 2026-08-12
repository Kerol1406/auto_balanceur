import numpy as np
import config

from Controllers.fuzzy_controller import FuzzyController
from robot import Robot
from scipy.optimize import differential_evolution


class FuzzyAutoTuner:
    """
    Auto-tuning du contrôleur flou par differential_evolution.

    La table de règles et les fonctions d'appartenance restent fixes (c'est
    l'expertise encodée dans le carnet). On optimise les paramètres continus
    des DEUX boucles floues de la cascade (flou dans flou) :
      Boucle interne (angle -> commande moteur) :
      - k_angle, k_vitesse : gains d'entrée (dilatent l'univers de discours)
      - c1, c2 : centres des classes de sortie, symétriques [-c2, -c1, 0, c1, c2]
      Boucle externe (position -> angle cible) :
      - pk_x, pk_dx : gains d'entrée sur [x, dx]
      - pc1, pc2 : centres de sortie (angles cibles, rad)

    La fonction de coût reprend les termes du LQRAutoTuner et ajoute une
    pénalité sur l'oscillation résiduelle (amplitude de theta et x sur le
    dernier quart de la simulation) pour éliminer le cycle limite.
    """

    def __init__(self, initial_state, target_state, sim_time=20.0, u_max=None):
        """
        Args:
            initial_state: État initial [x, x_dot, theta, theta_dot]
            target_state: État cible [x, x_dot, theta, theta_dot]
            sim_time: Durée de simulation (s)
            u_max: Saturation de la commande (rapport cyclique PWM)
        """
        if u_max is None:
            u_max = config.PWM_MAX
        # États initiaux supplémentaires pour l'évaluation robuste (voir
        # _cost_function). Optimiser sur une seule trajectoire permet à
        # l'algorithme de surapprendre ce cas precis : la solution obtenue peut
        # etre excellente sur cette trajectoire et mediocre partout ailleurs.
        self.etats_robustesse = [
            np.array([-0.3, 0.0, 0.15, 0.0]),
            np.array([0.0, 0.0, 0.05, 0.0]),
        ]
        self.initial_state = np.array(initial_state)
        self.target_state = np.array(target_state)
        self.sim_time = sim_time
        self.u_max = u_max
        self.best_cost = np.inf
        self.eval_count = 0

    def _simulate(self, params, initial_state=None):
        """
        Simule le robot avec la cascade floue paramétrée.
        params = [k_angle, k_vitesse, c1, c2, pk_x, pk_dx, pc1, pc2]
        initial_state : état de départ (par défaut celui du tuner)

        Returns:
            dict de métriques, ou dict avec 'crash_time' si chute.
        """
        k_angle, k_velocity, c1, c2, pk_x, pk_dx, pc1, pc2 = params
        controller = FuzzyController(
            state=[0.0, 0.0],
            output_centers=np.array([-c2, -c1, 0.0, c1, c2]),
            input_gains=(k_angle, k_velocity)
        )

        # Boucle externe de position floue, identique à main_controller
        fuzzy_pos = FuzzyController(
            state=[0.0, 0.0],
            output_centers=np.array([-pc2, -pc1, 0.0, pc1, pc2]),
            input_gains=(pk_x, pk_dx)
        )

        bot = Robot()
        state = (self.initial_state if initial_state is None else initial_state).copy()
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
        x_tol = 0.05       # 5 cm

        # Trajectoires pour mesurer l'oscillation résiduelle (dernier quart)
        theta_traj = np.empty(steps)
        x_traj = np.empty(steps)

        for i in range(steps):
            # Même câblage que dans main_controller (cascade floue + conversion deg + signes)
            target_theta = fuzzy_pos.compute([state[0], -state[1]])
            target_theta = np.clip(target_theta, -config.FUZZY_TARGET_THETA_MAX,
                                   config.FUZZY_TARGET_THETA_MAX)
            u = -controller.compute([np.degrees(state[2] - target_theta), -state[3]])
            u = np.clip(u, -self.u_max, self.u_max)

            state = bot.step(state, u, config.dt)
            theta_traj[i] = state[2]
            x_traj[i] = state[0]

            # Crash
            if abs(state[2]) > np.pi / 2:
                return {'crash_time': i * config.dt}

            t = i * config.dt
            err_theta = abs(state[2] - self.target_state[2])
            err_x = abs(state[0] - self.target_state[0])

            total_angle_error += err_theta ** 2
            total_pos_error += err_x ** 2
            total_effort += u ** 2

            max_overshoot_theta = max(max_overshoot_theta, err_theta)
            max_overshoot_x = max(max_overshoot_x, err_x)

            # Temps de stabilisation (dernier instant où on dépasse la tolérance)
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

        # Oscillation résiduelle sur le dernier quart de la simulation.
        # On utilise l'écart-type plutôt que l'amplitude crête-à-crête : cette
        # dernière est une métrique fragile (un seul point suffit à la faire
        # varier), ce qui permettait à l'optimiseur d'exploiter des optima en
        # lame de rasoir — des jeux de paramètres dont la performance s'effondre
        # dès que l'on modifie une valeur de 0.1 %, donc inutilisables sur un
        # robot réel. L'écart-type varie continûment avec les paramètres.
        tail = slice(int(0.75 * steps), steps)
        osc_theta = float(np.std(theta_traj[tail]))
        osc_x = float(np.std(x_traj[tail]))
        mean_x_tail = float(np.abs(x_traj[tail].mean()))
        # Conservées pour l'affichage : ce que l'on mesurerait à l'oscilloscope
        osc_theta_crete = float(theta_traj[tail].max() - theta_traj[tail].min())
        osc_x_crete = float(x_traj[tail].max() - x_traj[tail].min())

        return {
            'osc_theta': osc_theta,
            'osc_x': osc_x,
            'osc_theta_crete': osc_theta_crete,
            'osc_x_crete': osc_x_crete,
            'mean_x_tail': mean_x_tail,
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
        params = [k_angle, k_vitesse, c1, c2, pk_x, pk_dx, pc1, pc2]
        """
        self.eval_count += 1

        # Les centres doivent rester ordonnés : c2 > c1 (pour chaque boucle)
        if params[3] <= params[2] or params[7] <= params[6]:
            return 1e7

        # Évaluation sur PLUSIEURS états initiaux. Optimiser sur une seule
        # trajectoire laisse l'algorithme surapprendre ce cas precis : la
        # solution peut etre excellente dessus et mediocre partout ailleurs.
        # On retient le coût le PIRE des cas, ce qui pousse vers des reglages
        # qui fonctionnent partout plutot que vers un optimum etroit.
        etats = [self.initial_state] + self.etats_robustesse
        couts = []
        resultat_principal = None

        for k, etat in enumerate(etats):
            result = self._simulate(params, initial_state=etat)

            if 'crash_time' in result:
                # Pénalité graduée : plus le robot survit longtemps, plus le coût
                # baisse, pour guider l'évolution vers les zones stables.
                return 1e5 + 1e5 * (1.0 - result['crash_time'] / self.sim_time)

            # Coût pondéré : termes du LQRAutoTuner + pénalité forte sur
            # l'oscillation résiduelle (le "dandinement" en régime permanent)
            couts.append(
                10.0 * result['settling_time_theta']       # Convergence rapide de theta
                + 5.0 * result['settling_time_x']          # Convergence rapide de x
                + 50.0 * result['total_angle_error']       # Erreur angulaire cumulée
                + 10.0 * result['total_pos_error']         # Erreur position cumulée
                + 0.01 * result['total_effort']            # Économie d'énergie (faible poids)
                + 20.0 * result['max_overshoot_theta']     # Pénaliser l'overshoot angulaire
                + 300.0 * result['osc_theta']              # Oscillation résiduelle d'angle (ecart-type, rad)
                + 100.0 * result['osc_x']                  # Oscillation résiduelle de position (m)
                + 300.0 * result['mean_x_tail']            # Dérive résiduelle de position (m)
            )
            if k == 0:
                resultat_principal = result

        cost = max(couts)

        if cost < self.best_cost:
            self.best_cost = cost
            r = resultat_principal
            print(f"  [#{self.eval_count}] Nouveau meilleur coût: {cost:.2f} | "
                  f"angle: g=({params[0]:.2f}, {params[1]:.2f}) c=(±{params[2]:.2f}, ±{params[3]:.2f}) | "
                  f"pos: g=({params[4]:.1f}, {params[5]:.1f}) c=(±{params[6]:.3f}, ±{params[7]:.3f}) | "
                  f"osc_θ={np.degrees(r['osc_theta_crete']):.2f}° osc_x={r['osc_x_crete']:.3f}m")

        return cost

    def optimize(self, bounds=None, maxiter=15, seed=42):
        """
        Lance l'optimisation par differential_evolution.

        Args:
            bounds: Bornes [(min, max)] pour [k_angle, k_vitesse, c1, c2]
            maxiter: Nombre max de générations
            seed: Graine aléatoire pour reproductibilité

        Returns:
            dict: output_centers, input_gains optimaux et métriques
        """
        if bounds is None:
            bounds = [
                (0.2, 6.0),      # k_angle
                (0.2, 6.0),      # k_vitesse
                (0.005, 0.5),    # c1 (centres GD/DD, rapport cyclique)
                (0.02, 1.0),     # c2 (centres GV/DV, rapport cyclique)
                (0.5, 40.0),     # pk_x  : x (m) -> univers angle (±30)
                (0.2, 20.0),     # pk_dx : dx (m/s) -> univers vitesse (±5)
                (0.005, 0.15),   # pc1 (angles cibles doux, rad)
                (0.02, 0.30),    # pc2 (angles cibles forts, rad ; clip à 0.17 ensuite)
            ]

        print("=" * 60)
        print("FUZZY AUTO-TUNER (differential_evolution)")
        print("=" * 60)
        print(f"État initial : {self.initial_state}")
        print(f"État cible   : {self.target_state}")
        print(f"Bornes       : {bounds}")
        print(f"Max itérations: {maxiter}")
        print("-" * 60)

        self.best_cost = np.inf
        self.eval_count = 0

        result = differential_evolution(
            self._cost_function,
            bounds=bounds,
            maxiter=maxiter,
            seed=seed,
            tol=1e-3,
            polish=True,
            disp=False,
        )

        k_angle, k_velocity, c1, c2, pk_x, pk_dx, pc1, pc2 = result.x
        output_centers = np.array([-c2, -c1, 0.0, c1, c2])
        input_gains = np.array([k_angle, k_velocity])
        pos_output_centers = np.array([-pc2, -pc1, 0.0, pc1, pc2])
        pos_input_gains = np.array([pk_x, pk_dx])

        # Simuler une dernière fois pour les métriques finales
        metrics = self._simulate(result.x)

        print("\n" + "=" * 60)
        print("RÉSULTAT OPTIMAL")
        print("=" * 60)
        print(f"Interne - gains [angle, vitesse] : ({k_angle:.4f}, {k_velocity:.4f})")
        print(f"Interne - centres de sortie      : {output_centers.round(4)}")
        print(f"Externe - gains [x, dx]          : ({pk_x:.4f}, {pk_dx:.4f})")
        print(f"Externe - centres (angles cibles): {pos_output_centers.round(4)}")
        print(f"Coût final : {result.fun:.4f}")
        print(f"Évaluations: {self.eval_count}")
        if 'crash_time' not in metrics:
            print(f"Temps stabilisation θ : {metrics['settling_time_theta']:.2f} s")
            print(f"Temps stabilisation x : {metrics['settling_time_x']:.2f} s")
            print(f"Overshoot max θ       : {np.degrees(metrics['max_overshoot_theta']):.1f}°")
            print(f"Overshoot max x       : {metrics['max_overshoot_x']:.3f} m")
            print(f"Oscillation résiduelle θ : {np.degrees(metrics['osc_theta_crete']):.2f}° "
                  f"crête-à-crête ({np.degrees(metrics['osc_theta']):.3f}° d'écart-type)")
            print(f"Oscillation résiduelle x : {metrics['osc_x_crete']:.3f} m "
                  f"crête-à-crête ({metrics['osc_x']:.4f} m d'écart-type)")
        print("=" * 60)

        return {
            'output_centers': output_centers,
            'input_gains': input_gains,
            'pos_output_centers': pos_output_centers,
            'pos_input_gains': pos_input_gains,
            'params': result.x,
            'cost': result.fun,
            'metrics': metrics,
        }


def update_config_file(result, config_path="config.py"):
    """
    Lit config.py, remplace les lignes FUZZY_... et sauvegarde.
    Les commentaires en fin de ligne sont préservés.
    """
    print(">>> Sauvegarde des paramètres flous optimaux dans config.py...")

    # 6 decimales : avec 4, l'arrondi suffisait a degrader sensiblement le
    # comportement quand l'optimum trouve etait etroit.
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
                # On conserve le commentaire de fin de ligne s'il y en a un
                commentaire = ""
                if "#" in line:
                    commentaire = "  " + line[line.index("#"):].rstrip()
                f.write(f"{nom} = {remplacements[nom]}{commentaire}\n")
            else:
                f.write(line)  # On réécrit les autres lignes à l'identique


def run_optimization(initial_state=None, target_state=None, sim_time=20.0,
                     maxiter=15, save_to_config=True):
    """Fonction principale : optimise et met à jour config.py."""
    if initial_state is None:
        # Le robot est posé quasi vertical avant d'être lâché
        initial_state = np.array([-1.0, 0.0, np.radians(10.0), 0.0])
    if target_state is None:
        target_state = np.zeros(4)

    tuner = FuzzyAutoTuner(initial_state, target_state, sim_time=sim_time)
    result = tuner.optimize(maxiter=maxiter)

    if save_to_config:
        update_config_file(result)

    return result


if __name__ == "__main__":
    run_optimization()
