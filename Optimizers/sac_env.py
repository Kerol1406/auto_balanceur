import numpy as np
import gymnasium as gym
from gymnasium import spaces

import config
from robot import Robot


class BalanceEnv(gym.Env):
    """
    Environnement Gymnasium utilisé pour entraîner SAC.

    Gym ajoute autour de robot.py:
        - action_space
        - observation_space
        - reset()
        - step()
        - reward
        - conditions d'arrêt
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        theta_fail=np.pi / 4,
        x_fail=0.5,
        max_episode_steps=None,
        init_theta_range=np.radians(5),
        init_theta_dot_range=1.0,
        reward_weights=None,
        domain_randomization=False,
        observation_noise_std=(0.01, 0.015, 0.015, 0.03),
        normalize_obs=True,
    ):
        super().__init__()

        if max_episode_steps is None:
            max_episode_steps = config.SAC_EPISODE_STEPS
        if reward_weights is None:
            reward_weights = config.SAC_REWARD_WEIGHTS

        self.dt = config.dt
        self.theta_fail = theta_fail
        self.x_fail = x_fail
        self.max_episode_steps = max_episode_steps
        self.init_theta_range = init_theta_range
        self.init_theta_dot_range = init_theta_dot_range
        self.domain_randomization = domain_randomization
        self.observation_noise_std = np.array(observation_noise_std, dtype=np.float32)
        self.normalize_obs = normalize_obs
        self.obs_bounds = np.asarray(config.SAC_OBS_BOUNDS, dtype=np.float32)

        (
            self.w_theta,
            self.w_theta_dot,
            self.w_x,
            self.w_x_dot,
            self.w_duty,
            self.w_delta_duty,
        ) = reward_weights

        self.bot = Robot()
        self.state = np.zeros(4, dtype=np.float64)
        self.step_count = 0
        self.last_duty = 0.0

        # TODO 1: définir l'espace d'action SAC.
        # Une action = un duty cycle dans [-1, 1].
        self.action_space = None

        # TODO 2: définir l'espace d'observation.
        # Une observation = [x, x_dot, theta, theta_dot].
        self.observation_space = None

    def _get_obs(self):
        obs = self.state.astype(np.float32)

        # TODO 3: si domain_randomization est actif, ajouter un bruit gaussien.
        # Indice: self.np_random.normal(0.0, self.observation_noise_std)

        # TODO 4: si normalize_obs est actif, diviser obs par self.obs_bounds.

        return obs

    def _get_info(self):
        return {
            "x": self.state[0],
            "theta": self.state[2],
            "theta_deg": np.degrees(self.state[2]),
            "duty": self.last_duty,
        }

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # TODO 5: tirer x0, theta0 et theta_dot0 aléatoirement.
        # x0 proche de 0, theta0 dans [-init_theta_range, init_theta_range].
        x0 = None
        theta0 = None
        theta_dot0 = None

        self.bot = Robot()
        self.state = np.array([x0, 0.0, theta0, theta_dot0], dtype=np.float64)
        self.step_count = 0
        self.last_duty = 0.0

        return self._get_obs(), self._get_info()

    def step(self, action):
        # TODO 6: convertir action[0] en duty saturé dans [-PWM_MAX, PWM_MAX].
        duty = None

        # TODO 7: faire avancer la physique avec robot.py.
        self.state = None
        self.step_count += 1

        x, x_dot, theta, theta_dot = self.state
        delta_duty = duty - self.last_duty

        # TODO 8: construire les erreurs normalisées.
        theta_error = None
        theta_dot_error = None
        x_error = None
        x_dot_error = None

        # TODO 9: calculer la reward.
        # Elle doit pénaliser angle, vitesse angulaire, position, vitesse x,
        # effort moteur et variation de commande.
        reward = None

        self.last_duty = duty

        # TODO 10: définir terminated et truncated.
        terminated = None
        truncated = None

        if terminated:
            reward -= 20.0

        return self._get_obs(), reward, terminated, truncated, self._get_info()
