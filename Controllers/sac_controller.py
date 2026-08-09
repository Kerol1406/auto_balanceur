import numpy as np

import config


class SACController:
    """
    Contrôleur SAC utilisé par MainController.

    Entrée:
        state = [x, x_dot, theta, theta_dot]

    Sortie:
        duty cycle PWM dans [-1, 1]
    """

    def __init__(self, policy_path=None, max_duty=None):
        try:
            from stable_baselines3 import SAC
        except ImportError as exc:
            raise ImportError(
                "Le contrôleur SAC nécessite stable-baselines3. "
                "Installe-le sur la machine d'entraînement/simulation."
            ) from exc

        if policy_path is None:
            policy_path = config.SAC_POLICY_PATH
        if max_duty is None:
            max_duty = config.SAC_MAX_DUTY

        # TODO 1: charger le modèle SAC depuis policy_path.
        # Indice: SAC.load(chemin)
        self.model = None

        # TODO 2: récupérer les bornes de normalisation depuis config.py.
        # Indice: np.asarray(..., dtype=np.float32)
        self.obs_bounds = None

        self.max_duty = float(max_duty)

    def compute(self, state):
        # TODO 3: convertir state en tableau float32.
        obs = None

        # TODO 4: normaliser l'observation avec self.obs_bounds.
        obs_normalisee = None

        # TODO 5: calculer l'action du modèle en mode déterministe.
        # Indice: self.model.predict(observation, deterministic=True)
        action = None

        # TODO 6: extraire le duty, puis saturer dans [-self.max_duty, self.max_duty].
        duty = None

        return duty
