import numpy as np
import config

class Robot:
    def __init__(self, mass_factor=1.0):
        # Chargement des paramètres physiques
        self.M = config.M
        self.m = config.m * mass_factor
        self.l = config.l
        self.R = config.R
        self.I = config.I
        self.J = config.J
        self.g = config.g
        self.bx = config.bx
        self.btheta = config.btheta

        # Constantes pré-calculées
        self.M_total = self.M + self.m + (self.J / self.R**2)
        self.I_total = self.I + self.m * (self.l**2)

    def derivarives(self, state, u, f_ext=0.0):
        """
        Calcule les dérivées de l'état (vitesses et accélérations).
        C'est ici que réside la PHYSIQUE (Lagrange).
        
        state : [x, dx, theta, dtheta]
        u     : tension/couple moteur (scalaire)
        
        Retourne : [dx, ddx, dtheta, ddtheta]
        """
        # 1. Extraction des variables pour lisibilité
        x, dx, theta, dtheta = state
        
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        # 2. Construction de la Matrice de Masse (M_mat)
        # [ M_total      m*l*cos(t) ]
        # [ m*l*cos(t)   I_total    ]
        m11 = self.M_total
        m12 = self.m * self.l * cos_t
        m21 = m12
        m22 = self.I_total
        
        M_mat = np.array([[m11, m12], 
                          [m21, m22]])

        # 3. Construction du vecteur des forces (RHS - Right Hand Side)
        # On regroupe : Coriolis, Gravité, Frottements, Moteur
        
        # Termes de Coriolis/Centrifuge (m*l*sin(t)*dtheta^2)
        coriolis = self.m * self.l * sin_t * (dtheta**2)
        
        # Terme de Gravité (m*g*l*sin(t))
        gravity = self.m * self.g * self.l * sin_t
        
        # Forces généralisées (Moteur et Frottements)
        # Equation en x : Force moteur - frottement + force centrifuge pendule
        Qx = (u / self.R) - (self.bx * dx) + coriolis + f_ext
        
        # Equation en theta : -Couple moteur - frottement + gravité
        Qtheta = -u - (self.btheta * dtheta) + gravity
        
        V_forces = np.array([Qx, Qtheta])

        # 4. Résolution du système linéaire : Accélérations = inv(M) * Forces
        # Cela nous donne [ddx, ddtheta]
        accels = np.linalg.solve(M_mat, V_forces)
        ddx = accels[0]
        ddtheta = accels[1]

        # 5. Retour du vecteur dérivé complet
        # [vitesse_x, accel_x, vitesse_theta, accel_theta]
        return np.array([dx, ddx, dtheta, ddtheta])

    def step(self, state, u, dt, f_ext=0.0):
        """
        Intégrateur Runge-Kutta 4 (RK4).
        Permet de passer de l'état à t, vers l'état à t+dt avec précision.
        """
        k1 = self.derivarives(state, u, f_ext)
        k2 = self.derivarives(state + 0.5 * dt * k1, u, f_ext)
        k3 = self.derivarives(state + 0.5 * dt * k2, u, f_ext)
        k4 = self.derivarives(state + dt * k3, u, f_ext)

        # Formule RK4 : moyenne pondérée des pentes
        new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        return new_state