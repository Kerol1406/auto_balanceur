import json
import os
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

import config
from Optimizers.sac_env import BalanceEnv


POLICY_KWARGS = dict(net_arch=[config.SAC_HIDDEN_SIZE, config.SAC_HIDDEN_SIZE])
STAGES = config.SAC_CURRICULUM
SAVE_PATH = config.SAC_POLICY_PATH
LOG_DIR = "Ressources/SAC/tensorboard_logs"
BEST_MODELS_DIR = "Ressources/SAC/best_models"


def set_learning_rate(model, lr):
    """Met à jour le learning rate d'un modèle SAC déjà créé."""
    model.learning_rate = lr
    for opt in (model.policy.actor.optimizer, model.policy.critic.optimizer):
        for group in opt.param_groups:
            # TODO 1: remplacer le learning rate du groupe d'optimiseur.
            group["lr"] = None
    if getattr(model, "ent_coef_optimizer", None) is not None:
        for group in model.ent_coef_optimizer.param_groups:
            # TODO 2: même mise à jour pour l'optimiseur d'entropie.
            group["lr"] = None


def save_experiment_config():
    config_data = {
        "algorithm": "SAC",
        "policy_kwargs": POLICY_KWARGS,
        "buffer_size": config.SAC_BUFFER_SIZE,
        "batch_size": config.SAC_BATCH_SIZE,
        "gamma": config.SAC_GAMMA,
        "tau": config.SAC_TAU,
        "action": "duty cycle PWM in [-1, 1]",
        "stages": [
            {"stage": i + 1, "theta_deg": deg, "domain_rand": dr,
             "learning_rate": lr, "steps": steps}
            for i, (deg, dr, lr, steps) in enumerate(STAGES)
        ],
    }
    os.makedirs("Ressources/SAC", exist_ok=True)
    with open("Ressources/SAC/experiment_config.json", "w") as f:
        json.dump(config_data, f, indent=4)


def make_env(theta_deg, domain_randomization):
    # TODO 3: créer BalanceEnv avec init_theta_range et domain_randomization,
    # puis l'envelopper dans Monitor(...).
    env = None
    return env


def main():
    save_experiment_config()
    model = None

    for i, (theta_deg, dr, lr, steps) in enumerate(STAGES):
        env = make_env(theta_deg, dr)
        eval_env = make_env(theta_deg, dr)
        stage_name = f"stage_{i+1}"
        stage_best_model_path = os.path.join(BEST_MODELS_DIR, stage_name)

        eval_callback = EvalCallback(
            eval_env=eval_env,
            best_model_save_path=stage_best_model_path,
            log_path=os.path.join(LOG_DIR, stage_name, "eval"),
            eval_freq=5_000,
            n_eval_episodes=5,
            deterministic=True,
            verbose=0,
        )

        if model is None:
            # TODO 4: créer le modèle SAC avec les paramètres de config.py.
            model = None
        else:
            # TODO 5: garder le même modèle mais changer son environnement,
            # puis adapter le learning rate au stage courant.
            pass

        print(f"--- Étape {i+1}/{len(STAGES)}: theta=+/-{theta_deg}deg, "
              f"bruit_capteur={dr}, lr={lr}, {steps} pas ---")

        # TODO 6: lancer model.learn(...), sans remettre le compteur de temps à zéro.

        # TODO 7: sauvegarder le modèle final du stage.

    # TODO 8: sauvegarder le modèle final dans SAVE_PATH.


if __name__ == "__main__":
    main()
