"""
Modèle physique du robot auto-balanceur (pendule inversé sur roues).

C'est le SIMULATEUR : il ne contient aucune commande, seulement la physique.
On lui donne un état et une commande moteur, il rend l'état à l'instant
suivant. Tous les contrôleurs du projet seront évalués contre ce fichier :
s'il est faux, tout le reste l'est aussi.

ÉTAT ET COMMANDE
----------------
state = [x, dx, theta, dtheta]   (voir Controllers/dummy_controller.py)
duty  = rapport cyclique PWM dans [-1, 1]

ÉQUATIONS (formalisme de Lagrange)
----------------------------------
En notant M_total = M + m + J/R^2 (masse en translation, roues comprises) et
I_total = I + m*l^2 (inertie du pendule ramenée à l'axe des roues), le système
s'écrit sous forme matricielle :

    [ M_total        m*l*cos(theta) ] [ ddx     ] = [ Qx     ]
    [ m*l*cos(theta) I_total        ] [ ddtheta ] = [ Qtheta ]

avec, au second membre :

    Qx     = u/R - bx*dx + m*l*sin(theta)*dtheta^2
             (force au sol, frottement, effet centrifuge du pendule)
    Qtheta = -u - btheta*dtheta + m*g*l*sin(theta)
             (réaction du couple moteur, frottement, gravité qui fait tomber)

où u est le COUPLE (N.m) délivré par les moteurs, obtenu à partir du rapport
cyclique via motor.py.

À COMPLÉTER : voir les TODO.
"""

import numpy as np

import config
from motor import Motor


class Robot:
    def __init__(self):
        # Chargement des paramètres physiques
        self.M = config.M
        self.m = config.m
        self.l = config.l
        self.R = config.R
        self.I = config.I
        self.J = config.J
        self.g = config.g
        self.bx = config.bx
        self.btheta = config.btheta

        # DONE 1 : constantes pré-calculées (voir l'en-tête du fichier)
        self.M_total = self.M + self.m + self.J/self.R**2  # M + m + J/R^2
        self.I_total = self.I + self.m*self.l**2   # I + m*l^2

        # Modèle des moteurs : convertit le rapport cyclique en couple
        self.motor = Motor()

    def couple_moteur(self, state, duty):
        """
        Couple délivré par les moteurs pour un rapport cyclique donné.

        Les moteurs sont montés ENTRE le châssis (qui bascule) et les roues.
        La vitesse vue par l'arbre moteur n'est donc pas
        la vitesse de rotation des roues dans le repère du sol, mais leur
        vitesse RELATIVE au corps.

        Returns:
            float: couple total (N.m)
        """
        # DONE 2 : calculer 
        # omega_relatif = (dx / R) - dtheta,
        omega_relatif = (state[1] / self.R) - state[3]
        #          puis renvoyer self.motor.torque(duty, omega_relatif)
        return self.motor.torque(duty, omega_relatif)

    def derivarives(self, state, duty):
        """
        Dérivées de l'état

        Args:
            state (array): [x, dx, theta, dtheta]
            duty (float): rapport cyclique PWM dans [-1, 1]

        Returns:
            np.ndarray: [dx, ddx, dtheta, ddtheta]
        """
        # DONE 3 : extraire x, dx, theta, dtheta de state
        x, dx, theta, dtheta = state

        # DONE 4 : convertir le rapport cyclique en couple avec couple_moteur()
        u = self.couple_moteur(state, duty)

        # DONE 5 : construire la matrice de masse 2x2 (voir l'en-tête)
        M_mat = np.array([[self.M_total, self.m*self.l*np.cos(theta)], 
                          [self.m*self.l*np.cos(theta), self.I_total]])
        
        # DONE 6 : construire le vecteur des forces généralisées [Qx, Qtheta]
        Qx = self.u/self.R - self.bx*dx + self.m*self.l*np.sin(theta)*dtheta**2
        Qtheta = -self.u - self.btheta*dtheta + self.m*self.g*self.l*np.sin(theta)
        V_forces = np.array([[Qx, Qtheta]])

        # DONE 7 : résoudre le système linéaire pour obtenir [ddx, ddtheta].
        #          Utiliser np.linalg.solve.
        accels = np.linalg.solve(M_mat, V_forces)
        ddx, ddtheta = accels        

        # DONE 8 : renvoyer np.array([dx, ddx, dtheta, ddtheta])
        return np.array([dx, ddx, dtheta, ddtheta])

    def step(self, state, duty, dt):
        """
        Avance la simulation d'un pas de temps par Runge-Kutta d'ordre 4 (RK4).

        Args:
            state (array): état à l'instant t
            duty (float): rapport cyclique appliqué pendant le pas
            dt (float): pas de temps (s)

        Returns:
            np.ndarray: état à l'instant t + dt
        """
        # DONE 9 : calculer les quatre pentes k1, k2, k3, k4
        #   k1 = f(state,                duty)
        k1 = self.derivarives(state, duty)
        #   k2 = f(state + dt/2 * k1,    duty)
        k2 = self.derivarives(state + dt*k1/2, duty)
        #   k3 = f(state + dt/2 * k2,    duty)
        k3 = self.derivarives(state + dt*k2/2, duty)
        #   k4 = f(state + dt   * k3,    duty)
        k4 = self.derivarives(state + dt*k3, duty)

        # DONE 10 : renvoyer state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

        return state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
