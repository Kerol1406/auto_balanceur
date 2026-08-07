"""
Contrôleur par apprentissage par renforcement : Soft Actor-Critic (SAC).

DE QUOI S'AGIT-IL ?
-------------------
Les trois autres contrôleurs du projet supposent que l'on connaît le robot :
le PID se règle sur son comportement, le LQR a besoin des matrices A et B, le
flou a besoin de l'expertise d'un pilote. Le SAC, lui, n'a besoin de RIEN : il
apprend la loi de commande tout seul, en essayant des actions et en observant
la récompense obtenue.

Ce fichier ne contient QUE la partie "utilisation" (inférence) : on charge une
politique déjà entraînée et on lui demande une action. L'entraînement, lui, est
dans Optimizers/sac_trainer.py.

    Optimizers/sac_trainer.py  -->  Ressources/sac_policy.pt  -->  ce fichier

POURQUOI SAC ET PAS Q-LEARNING ?
--------------------------------
La commande du robot est CONTINUE (un rapport cyclique dans [-1, 1]). Un
Q-learning tabulaire imposerait de discrétiser l'action et l'état, ce qui donne
une commande en escalier et une table qui explose en dimension. SAC travaille
directement en continu et, grâce à son terme d'entropie, explore beaucoup mieux :
il cherche la politique la plus performante ET la plus "indécise" possible, ce
qui évite de se figer trop tôt sur une mauvaise stratégie.

ARCHITECTURE
------------
    observation (4)  ->  réseau "acteur"  ->  moyenne + écart-type
                                              -> tanh -> action dans [-1, 1]

En inférence on prend la moyenne (comportement déterministe) ; pendant
l'entraînement on tire au hasard dans la gaussienne (exploration).

DÉPENDANCE : PyTorch (`pip install torch`).

À COMPLÉTER : voir les TODO.
"""

import numpy as np

import config


class GaussianPolicy:
    """
    Réseau acteur : observation -> distribution des actions.

    Cette classe doit hériter de `torch.nn.Module`. Elle est définie ici (et non
    dans le trainer) parce que l'entraînement ET l'utilisation en ont besoin.
    """

    def __init__(self, obs_dim=4, act_dim=1, hidden_size=None):
        # TODO 1 : appeler le constructeur parent, puis construire le réseau :
        #   - un tronc commun de deux couches denses (hidden_size neurones,
        #     activation ReLU) ; hidden_size par défaut = config.SAC_HIDDEN_SIZE
        #   - une tête "mu"       : hidden_size -> act_dim
        #   - une tête "log_std"  : hidden_size -> act_dim, dont la sortie sera
        #     bornée (typiquement dans [-20, 2]) pour éviter les explosions
        raise NotImplementedError("GaussianPolicy.__init__ : à implémenter")

    def forward(self, obs):
        """Retourne (mu, log_std) pour un lot d'observations."""
        # TODO 2 : passer dans le tronc puis les deux têtes, borner log_std
        raise NotImplementedError("GaussianPolicy.forward : à implémenter")

    def sample(self, obs):
        """
        Échantillonne une action et retourne aussi sa log-probabilité.

        Le "reparameterization trick" : on tire un bruit gaussien standard,
        a_brut = mu + std * bruit, puis on écrase dans [-1, 1] avec une tanh.
        Attention : la tanh déforme la densité, il faut corriger la
        log-probabilité du terme log(1 - tanh(a_brut)^2) — c'est LE détail que
        tout le monde oublie et qui fait diverger l'entraînement.

        Returns:
            (action, log_prob, mu_tanh)
        """
        # TODO 3 : implémenter l'échantillonnage reparamétré + correction tanh
        raise NotImplementedError("GaussianPolicy.sample : à implémenter")


class SACController:
    """Utilise une politique SAC entraînée comme contrôleur du robot."""

    def __init__(self, policy_path=None, deterministic=True, verbose=True):
        """
        Args:
            policy_path (str): fichier de la politique entraînée
                (config.SAC_POLICY_PATH par défaut)
            deterministic (bool): True = joue la moyenne de la gaussienne
                (comportement reproductible, à utiliser pour les mesures) ;
                False = échantillonne (utile pour visualiser l'exploration)
            verbose (bool): affiche un récapitulatif au chargement
        """
        self.policy_path = config.SAC_POLICY_PATH if policy_path is None else policy_path
        self.deterministic = deterministic
        self.verbose = verbose
        self.policy = None

        # TODO 4 : instancier la politique et charger les poids depuis
        #          policy_path (torch.load + load_state_dict), puis passer le
        #          réseau en mode évaluation (.eval()).
        #          Si le fichier n'existe pas, lever une erreur EXPLICITE qui
        #          renvoie l'utilisateur vers Optimizers/sac_trainer.py.
        raise NotImplementedError("SACController.__init__ : à implémenter")

    def observation(self, state, target_state=None):
        """
        Transforme l'état du robot en entrée du réseau.

        Deux précautions qui changent tout :
          - travailler sur l'ERREUR (state - target_state) et non sur l'état
            brut, sinon la politique n'apprend qu'à revenir en x = 0 ;
          - NORMALISER les quatre composantes pour qu'elles aient des ordres de
            grandeur comparables (un angle vaut 0.1 rad quand une vitesse vaut
            3 rad/s : sans normalisation, le réseau ignore l'angle).

        Le trainer DOIT utiliser exactement la même fonction, sinon la politique
        reçoit en simulation des entrées différentes de celles vues à
        l'entraînement.

        Returns:
            np.ndarray de taille 4
        """
        # TODO 5 : calculer l'erreur d'état et appliquer la normalisation
        raise NotImplementedError("SACController.observation : à implémenter")

    def compute(self, state, target_state=None):
        """
        Interface commune à tous les contrôleurs du projet.

        Args:
            state (array): [x, dx, theta, dtheta]
            target_state (array): état visé

        Returns:
            float: rapport cyclique PWM dans [-1, 1]
        """
        # TODO 6 : construire l'observation, la convertir en tenseur, appeler
        #          le réseau SANS calcul de gradient (torch.no_grad()), puis
        #          renvoyer un float Python saturé dans [-config.PWM_MAX, config.PWM_MAX]
        raise NotImplementedError("SACController.compute : à implémenter")


if __name__ == "__main__":
    # Test rapide une fois la politique entraînée
    controleur = SACController()
    etat = np.array([0.0, 0.0, -0.2, 0.0])
    print(f"Action pour theta = -0.2 rad : u = {controleur.compute(etat):.4f}")
