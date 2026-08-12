"""
Banc de benchmark comparatif : PID / LQR / FUZZY / SAC (à venir).

Script autonome (aucune modification des fichiers existants). Il rejoue
plusieurs scénarios sur chaque contrôleur, calcule des métriques de
performance normalisées, et écrit un rapport Markdown + des figures PNG
dans Ressources/Benchmark/.

Usage :
    python benchmark.py
    python benchmark.py --controllers PID LQR
    python benchmark.py --sim-time 10
"""

import argparse
import os
import time
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import config
from robot import Robot
from Controllers.pid_controller import PIDCascade
from Controllers.lqr_controller import LQR
from Controllers.fuzzy_controller import FuzzyController


OUTPUT_DIR = os.path.join("Ressources", "Benchmark")
FIGURES_DIR = os.path.join(OUTPUT_DIR, "Figures")
REPORT_PATH = os.path.join(OUTPUT_DIR, "benchmark_report.md")

SIM_TIME = 15.0
THETA_TOL = 0.01   # rad ~ 0.6°, tolérance pour le temps de stabilisation
X_TOL = 0.05       # m

# Écarts-types du bruit de mesure ajouté à l'état AVANT qu'il n'atteigne le
# contrôleur (le robot réel n'a jamais accès à l'état exact, seulement à ce
# que rendent ses capteurs). Ordres de grandeur plausibles pour la plateforme :
#   x      : résolution des encodeurs de roue                     -> 1 mm
#   dx     : vitesse dérivée des encodeurs, donc plus bruitée      -> 2 cm/s
#   theta  : fusion gyro/accéléro (type MPU6050)                   -> 0.5°
#   dtheta : mesure gyro brute                                     -> 0.05 rad/s
SENSOR_NOISE_STD = np.array([0.001, 0.02, np.radians(0.5), 0.05])
NOISE_SEED = 42   # fixe pour que tous les contrôleurs subissent le même bruit

SCENARIOS = [
    {"key": "nominal", "label": "Nominal",
     "description": "Léger déséquilibre initial (-0.20 rad), consigne à l'origine.",
     "state": [0.0, 0.0, -0.20, 0.0], "target_x": 0.0},
    {"key": "recul_15deg", "label": "Recul + 15°",
     "description": "Chariot reculé de 30 cm avec le pendule penché à 15°.",
     "state": [-0.3, 0.0, 0.15, 0.0], "target_x": 0.0},
    {"key": "faible_5deg", "label": "Faible perturbation",
     "description": "Angle initial faible (+5°) : cas facile de référence.",
     "state": [0.0, 0.0, 0.05, 0.0], "target_x": 0.0},
    {"key": "consigne_deplacement", "label": "Changement de consigne",
     "description": "Départ à l'équilibre, consigne de position déplacée à 0.5 m.",
     "state": [0.0, 0.0, 0.0, 0.0], "target_x": 0.5},
    {"key": "quasi_chute", "label": "Quasi-chute",
     "description": "Angle initial proche du seuil de chute (75% de CHUTE_THETA_MAX) : robustesse en cas extrême.",
     "state": [0.0, 0.0, 0.75 * config.CHUTE_THETA_MAX, 0.0], "target_x": 0.0},
    {"key": "bruit_capteurs", "label": "Bruit capteurs",
     "description": "Même état nominal que le premier scénario, mais les contrôleurs ne reçoivent "
                     "qu'une mesure bruitée de l'état (imperfection des encodeurs et du gyro/IMU) "
                     "au lieu de l'état exact simulé.",
     "state": [0.0, 0.0, -0.20, 0.0], "target_x": 0.0, "noise": True},
]


# ---------------------------------------------------------------------------
# Adaptateurs de contrôleurs
#
# Chaque fabrique expose loi(state) -> u (rapport cyclique PWM) et reset().
# Ils encapsulent ICI les différences d'interface entre PID (retourne un
# tuple), LQR (compute(state, target_state)) et FUZZY (cascade de deux
# instances combinées manuellement), pour ne modifier aucun fichier existant.
# ---------------------------------------------------------------------------

def _make_pid(target_x):
    cascade = PIDCascade(config.PID_Kp, config.PID_Ki, config.PID_Kd,
                          config.PID_Pos_Kp, config.PID_Pos_Ki, config.PID_Pos_Kd,
                          config.dt)

    def loi(state):
        u, _ = cascade.compute(state, target_x=target_x)
        return u

    return loi, cascade.reset


def _make_lqr(target_x):
    ctrl = LQR(Q=config.LQR_Q, R=config.LQR_R, verbose=False)
    target_state = np.array([target_x, 0.0, 0.0, 0.0])

    def loi(state):
        return ctrl.compute(state, target_state)

    return loi, (lambda: None)


def _make_fuzzy(target_x):
    interne = FuzzyController(state=[0.0, 0.0],
                               output_centers=config.FUZZY_OUTPUT_CENTERS,
                               input_gains=config.FUZZY_INPUT_GAINS)
    externe = FuzzyController(state=[0.0, 0.0],
                               output_centers=config.FUZZY_POS_OUTPUT_CENTERS,
                               input_gains=config.FUZZY_POS_INPUT_GAINS)

    def loi(state):
        # Généralisation par rapport à validation.py : la boucle externe voit
        # l'ERREUR (x - target_x) et non x brut, pour supporter un changement
        # de consigne comme le font nativement PID (target_x) et LQR (target_state).
        cible = np.clip(externe.compute([state[0] - target_x, -state[1]]),
                         -config.FUZZY_TARGET_THETA_MAX, config.FUZZY_TARGET_THETA_MAX)
        return -interne.compute([np.degrees(state[2] - cible), -state[3]])

    return loi, (lambda: None)


def _make_sac(target_x):
    """
    SAC (Soft Actor-Critic) : non encore implémenté dans ce projet.
    Import optionnel pour que ce banc l'intègre automatiquement dès qu'il
    existera, sans modification de benchmark.py. Interface attendue :
    Controllers/sac_controller.py exposant une classe SACController avec une
    méthode .compute(state) -> u (float, PWM dans [-1, 1]) et, si besoin,
    .reset(). Tant qu'il n'existe pas (ou échoue à s'instancier), le
    contrôleur est simplement absent du rapport.
    """
    try:
        from Controllers.sac_controller import SACController
    except ImportError:
        return None, None

    try:
        ctrl = SACController()
    except Exception as exc:
        print(f"[benchmark] SAC détecté mais non instanciable ({exc}) — ignoré.")
        return None, None

    def loi(state):
        return ctrl.compute(state)

    reset = getattr(ctrl, "reset", lambda: None)
    return loi, reset


CONTROLLER_FACTORIES = {
    "PID": _make_pid,
    "LQR": _make_lqr,
    "FUZZY": _make_fuzzy,
    "SAC": _make_sac,
}


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def simulate(nom, target_x, initial_state, sim_time=SIM_TIME, noise=False):
    """Rejoue un scénario pour un contrôleur. Retourne None si le contrôleur
    n'est pas disponible (import échoué), un dict de métriques sinon.

    Si noise=True, le contrôleur ne voit qu'une mesure bruitée de l'état
    (SENSOR_NOISE_STD) : la physique et les métriques, elles, restent
    calculées sur l'état EXACT — comme sur le robot réel, où seule la
    commande dépend de ce que rendent les capteurs, pas la dynamique."""
    factory = CONTROLLER_FACTORIES.get(nom)
    if factory is None:
        raise ValueError(f"Contrôleur inconnu : {nom}")

    loi, reset = factory(target_x)
    if loi is None:
        return None
    reset()

    # Seed fixe : les 4 contrôleurs subissent exactement le même bruit sur un
    # même scénario, pour que les écarts de performance viennent d'eux et non
    # d'un tirage aléatoire différent.
    rng = np.random.default_rng(NOISE_SEED) if noise else None

    bot = Robot()
    state = np.array(initial_state, dtype=float)
    steps = int(sim_time / config.dt)

    theta_traj = np.empty(steps)
    x_traj = np.empty(steps)
    u_traj = np.empty(steps)
    compute_times = np.empty(steps)

    effort = 0.0
    echec = None
    settling_theta = sim_time
    settling_x = sim_time
    overshoot_theta = 0.0
    overshoot_x = 0.0
    n_done = steps

    for i in range(steps):
        mesure = state if rng is None else state + rng.normal(0.0, SENSOR_NOISE_STD)

        t0 = time.perf_counter()
        u = loi(mesure)
        compute_times[i] = time.perf_counter() - t0

        u = float(np.clip(u, -config.PWM_MAX, config.PWM_MAX))
        state = bot.step(state, u, config.dt)

        theta_traj[i] = state[2]
        x_traj[i] = state[0]
        u_traj[i] = u
        effort += u * u * config.dt

        err_theta = abs(state[2])
        err_x = abs(state[0] - target_x)
        overshoot_theta = max(overshoot_theta, err_theta)
        overshoot_x = max(overshoot_x, err_x)
        if err_theta > THETA_TOL:
            settling_theta = (i + 1) * config.dt
        if err_x > X_TOL:
            settling_x = (i + 1) * config.dt

        if abs(state[2]) > config.CHUTE_THETA_MAX:
            echec = "chute"
            n_done = i + 1
            break
        if abs(state[0]) > config.CHUTE_X_MAX:
            echec = "dérive"
            n_done = i + 1
            break

    theta_traj = theta_traj[:n_done]
    x_traj = x_traj[:n_done]
    u_traj = u_traj[:n_done]
    compute_times = compute_times[:n_done]
    tail = slice(int(0.75 * n_done), n_done)

    return {
        "controller": nom,
        "echec": echec,
        "t_fin": n_done * config.dt,
        "time_axis": np.arange(n_done) * config.dt,
        "theta_traj": theta_traj,
        "x_traj": x_traj,
        "u_traj": u_traj,
        "x_max": float(np.abs(x_traj).max()),
        "x_fin": float(x_traj[-1]),
        "theta_fin_deg": float(np.degrees(theta_traj[-1])),
        "osc_theta_deg": float(np.degrees(theta_traj[tail].std())),
        "effort": effort,
        "settling_time_theta": settling_theta if echec is None else float("nan"),
        "settling_time_x": settling_x if echec is None else float("nan"),
        "overshoot_theta_deg": float(np.degrees(overshoot_theta)),
        "overshoot_x": overshoot_x,
        "mean_compute_us": float(compute_times.mean() * 1e6),
    }


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def plot_scenario(scenario, runs, path):
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for run in runs:
        if run is None:
            continue
        axes[0].plot(run["time_axis"], np.degrees(run["theta_traj"]), label=run["controller"])
        axes[1].plot(run["time_axis"], run["x_traj"], label=run["controller"])
        axes[2].plot(run["time_axis"], run["u_traj"], label=run["controller"])

    seuil_deg = np.degrees(config.CHUTE_THETA_MAX)
    axes[0].axhline(seuil_deg, color="r", ls="--", lw=0.8)
    axes[0].axhline(-seuil_deg, color="r", ls="--", lw=0.8)
    axes[0].set_ylabel("theta (deg)")
    axes[1].axhline(scenario["target_x"], color="grey", ls=":", lw=0.8)
    axes[1].set_ylabel("x (m)")
    axes[2].set_ylabel("u (PWM)")
    axes[2].set_xlabel("temps (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(f"Scénario : {scenario['label']}")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_synthese(all_results, scenarios, controllers, path):
    efforts, settlings, oscs = [], [], []
    for nom in controllers:
        runs_ok = [all_results[nom][s["key"]] for s in scenarios
                   if all_results[nom][s["key"]] is not None
                   and all_results[nom][s["key"]]["echec"] is None]
        efforts.append(np.mean([r["effort"] for r in runs_ok]) if runs_ok else np.nan)
        settlings.append(np.mean([r["settling_time_theta"] for r in runs_ok]) if runs_ok else np.nan)
        oscs.append(np.mean([r["osc_theta_deg"] for r in runs_ok]) if runs_ok else np.nan)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    axes[0].bar(controllers, efforts, color="steelblue")
    axes[0].set_title("Effort moyen (Σu²·dt)")
    axes[1].bar(controllers, settlings, color="darkorange")
    axes[1].set_title("Stabilisation moyenne (s)")
    axes[2].bar(controllers, oscs, color="seagreen")
    axes[2].set_title("Oscillation résiduelle moyenne (°)")
    for ax in axes:
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Synthèse globale (moyenne sur les scénarios réussis)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Rapport Markdown
# ---------------------------------------------------------------------------

def _fmt(value, spec="{:.3f}"):
    if value is None or (isinstance(value, float) and np.isnan(value)):
        return "—"
    return spec.format(value)


def write_report(all_results, scenarios, controllers_demandes, controllers_actifs,
                  sim_time, report_path, figures_dir):
    lines = []
    lines.append("# Rapport de benchmark — Pendule inversé sur chariot")
    lines.append("")
    lines.append(f"Généré le {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} par `benchmark.py`.")
    lines.append("")
    lines.append("## Méthodologie")
    lines.append("")
    lines.append(f"- Pas de simulation : `dt = {config.dt}` s ({1.0 / config.dt:.0f} Hz), intégration RK4 (`robot.py`)")
    lines.append(f"- Durée simulée par scénario : {sim_time:.1f} s")
    lines.append(f"- Échec si |theta| > {np.degrees(config.CHUTE_THETA_MAX):.1f}° (chute) ou |x| > {config.CHUTE_X_MAX} m (dérive)")
    lines.append(f"- Stabilisé si |theta| < {np.degrees(THETA_TOL):.2f}° et |x - consigne| < {X_TOL} m")
    lines.append("- Commande : rapport cyclique PWM ∈ [-1, 1] (modèle moteur CC avec force contre-électromotrice, `motor.py`)")
    lines.append("- Effort = Σ u²·dt (proxy d'énergie de commande)")
    lines.append("- Temps de calcul : mesuré en Python sur cette machine — indicatif, comparatif entre contrôleurs uniquement")
    lines.append(f"- Bruit capteurs (scénario dédié) : écarts-types gaussiens ajoutés à la mesure transmise au "
                  f"contrôleur (état physique inchangé) — x: {SENSOR_NOISE_STD[0]*1000:.1f} mm, "
                  f"dx: {SENSOR_NOISE_STD[1]*100:.1f} cm/s, theta: {np.degrees(SENSOR_NOISE_STD[2]):.2f}°, "
                  f"dtheta: {SENSOR_NOISE_STD[3]:.3f} rad/s ; même tirage (seed={NOISE_SEED}) pour tous les contrôleurs")
    lines.append("")
    lines.append("Contrôleurs demandés : " + ", ".join(controllers_demandes))
    indisponibles = [c for c in controllers_demandes if c not in controllers_actifs]
    if "SAC" in indisponibles:
        lines.append("")
        lines.append("> **SAC (Soft Actor-Critic)** : non encore implémenté dans le projet — absent de ce rapport. "
                      "`benchmark.py` le détectera et l'intégrera automatiquement dès que "
                      "`Controllers/sac_controller.py` existera (voir la docstring de `_make_sac`).")
    lines.append("")

    fig_rel = os.path.relpath(figures_dir, os.path.dirname(report_path) or ".").replace("\\", "/")

    for scenario in scenarios:
        lines.append(f"## Scénario : {scenario['label']}")
        lines.append("")
        lines.append(scenario["description"])
        lines.append("")
        lines.append(f"![{scenario['label']}]({fig_rel}/{scenario['key']}.png)")
        lines.append("")
        lines.append("| Contrôleur | Statut | t_fin (s) | \\|x\\|max (m) | x_fin (m) | theta_fin (°) | "
                      "osc_theta (°) | overshoot_theta (°) | stabil._theta (s) | stabil._x (s) | effort | calcul (µs) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
        for nom in controllers_demandes:
            r = all_results[nom][scenario["key"]]
            if r is None:
                lines.append(f"| {nom} | non disponible | — | — | — | — | — | — | — | — | — | — |")
                continue
            statut = "OK" if r["echec"] is None else f"ÉCHEC ({r['echec']})"
            lines.append(
                f"| {nom} | {statut} | {r['t_fin']:.2f} | {r['x_max']:.3f} | {r['x_fin']:+.3f} | "
                f"{r['theta_fin_deg']:+.2f} | {r['osc_theta_deg']:.3f} | {r['overshoot_theta_deg']:.2f} | "
                f"{_fmt(r['settling_time_theta'])} | {_fmt(r['settling_time_x'])} | "
                f"{r['effort']:.3f} | {r['mean_compute_us']:.2f} |"
            )
        lines.append("")

    lines.append("## Synthèse globale")
    lines.append("")
    lines.append(f"![Synthèse globale]({fig_rel}/synthese_globale.png)")
    lines.append("")
    lines.append("Moyennes calculées uniquement sur les scénarios réussis (sans chute ni dérive).")
    lines.append("")
    lines.append("| Contrôleur | Scénarios réussis | effort moyen | stabil._theta moyen (s) | osc_theta moyenne (°) | calcul moyen (µs) |")
    lines.append("|---|---|---|---|---|---|")
    for nom in controllers_demandes:
        runs = [all_results[nom][s["key"]] for s in scenarios]
        n_total = sum(1 for r in runs if r is not None)
        runs_ok = [r for r in runs if r is not None and r["echec"] is None]
        if n_total == 0:
            lines.append(f"| {nom} | non disponible | — | — | — | — |")
            continue
        if not runs_ok:
            lines.append(f"| {nom} | 0/{n_total} | — | — | — | — |")
            continue
        effort_moy = np.mean([r["effort"] for r in runs_ok])
        settl_moy = np.mean([r["settling_time_theta"] for r in runs_ok])
        osc_moy = np.mean([r["osc_theta_deg"] for r in runs_ok])
        calc_moy = np.mean([r["mean_compute_us"] for r in runs_ok])
        lines.append(f"| {nom} | {len(runs_ok)}/{n_total} | {effort_moy:.3f} | {settl_moy:.2f} | {osc_moy:.3f} | {calc_moy:.2f} |")
    lines.append("")

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Banc de benchmark des contrôleurs du pendule inversé.")
    parser.add_argument("--controllers", nargs="+", default=["PID", "LQR", "FUZZY", "SAC"],
                         help="Contrôleurs à évaluer (défaut : PID LQR FUZZY SAC).")
    parser.add_argument("--sim-time", type=float, default=SIM_TIME,
                         help=f"Durée simulée par scénario, en secondes (défaut : {SIM_TIME}).")
    args = parser.parse_args()

    controllers_demandes = [c.upper() for c in args.controllers]
    os.makedirs(FIGURES_DIR, exist_ok=True)

    all_results = {nom: {} for nom in controllers_demandes}
    controllers_actifs = []

    for scenario in SCENARIOS:
        runs = []
        for nom in controllers_demandes:
            r = simulate(nom, scenario["target_x"], scenario["state"], sim_time=args.sim_time,
                         noise=scenario.get("noise", False))
            all_results[nom][scenario["key"]] = r
            runs.append(r)
            if r is not None and nom not in controllers_actifs:
                controllers_actifs.append(nom)
        plot_scenario(scenario, runs, os.path.join(FIGURES_DIR, f"{scenario['key']}.png"))
        print(f"Scénario '{scenario['label']}' simulé.")

    if controllers_actifs:
        plot_synthese(all_results, SCENARIOS, controllers_actifs,
                       os.path.join(FIGURES_DIR, "synthese_globale.png"))

    write_report(all_results, SCENARIOS, controllers_demandes, controllers_actifs,
                 args.sim_time, REPORT_PATH, FIGURES_DIR)
    print(f"\nRapport écrit dans '{REPORT_PATH}'.")


if __name__ == "__main__":
    main()
