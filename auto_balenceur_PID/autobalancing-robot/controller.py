import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        
        # Mémoire pour l'intégrale et la dérivée
        self.integral = 0
        self.prev_error = 0

    def compute(self, target, current):
        """
        Calcule la commande u en fonction de la cible et de la valeur actuelle.
        """
        error = target - current
        
        # Terme Proportionnel
        P = self.Kp * error
        
        # Terme Intégral (somme des erreurs)
        self.integral += error * self.dt
        I = self.Ki * self.integral
        
        # Terme Dérivé (variation de l'erreur)
        # Note : error - prev_error est l'inverse de la vitesse
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        
        # Mise à jour pour le tour suivant
        self.prev_error = error
        
        return P + I + D


class EnergyController:
    """
    Contrôleur: commande par énergie     
    
    """
    
    def __init__(self, m, M, l, g, dt, K_lqr=None, k_energy=2.0, theta_threshold=0.2, smooth_transition=True):
        """
        Initialise le contrôleur par énergie (LQR temporairement désactivé).
        
        Args:
            m (float): Masse du pendule (kg)
            M (float): Masse du chariot (kg) 
            l (float): Longueur du pendule (m)
            g (float): Accélération gravitationnelle (m/s²)
            dt (float): Pas de temps de simulation (s)
            K_lqr (array-like): Gains LQR [K_x, K_x_dot, K_theta, K_theta_dot] - DÉSACTIVÉ
            k_energy (float): Gain pour la commande par énergie
            theta_threshold (float): Seuil de basculement entre modes (rad) - DÉSACTIVÉ
            smooth_transition (bool): Active la transition douce entre modes - DÉSACTIVÉ
        """
        # Paramètres physiques
        self.m = m
        self.M = M  
        self.l = l
        self.g = g
        self.dt = dt
        
        # Paramètres de contrôle
        # self.K_lqr = np.array(K_lqr) if K_lqr is not None else np.zeros(4)  # Gains LQR (1x4) - DÉSACTIVÉ
        self.k_energy = k_energy
        # self.theta_threshold = theta_threshold  # DÉSACTIVÉ
        # self.smooth_transition = smooth_transition  # DÉSACTIVÉ
        
        # Énergie désirée à l'équilibre (pendule en haut)
        self.E_des = self.m * self.g * self.l
        # self.E_des = 0
        # Variables internes
        self.reset()
    
    def reset(self):
        """
        Réinitialise les variables internes du contrôleur.
        """
        # Historique pour le débogage (optionnel)
        self.last_mode = None
        self.energy_history = []
    
    def _compute_energy(self, theta, theta_dot):
        """
        Calcule l'énergie totale du pendule.
        
        Args:
            theta (float): Angle du pendule (rad)
            theta_dot (float): Vitesse angulaire (rad/s)
    
        Returns:
            float: Énergie totale du pendule
        """
        # Énergie cinétique + énergie potentielle
        # E_kinetic = 0.5 * self.m * (self.l**2) * (theta_dot**2)
        # E_potential = self.m * self.g * self.l * (1 - np.cos(theta))

        omega = np.sqrt(self.g / self.l)
        E = self.m* self.g * self.l * (0.5 * (theta_dot / omega)**2 + np.cos(theta) - 1 )
        print(f"Energie calculée pour theta = {theta * 180/np.pi} °C: {E} ")
        # return E_kinetic + E_potential
        return 1.2*E
    
    def _compute_energy_command(self, theta, theta_dot):
        """
        Calcule la commande par énergie pour le swing-up.
        
        Args:
            theta (float): Angle du pendule (rad)
            theta_dot (float): Vitesse angulaire (rad/s)
            
        Returns:
            float: Commande par énergie
        """
        # Énergie actuelle
        E_current = self._compute_energy(theta, theta_dot)
        
        # Erreur d'énergie
        E_error = self.E_des - E_current
        
        # Commande par énergie avec direction basée sur le signe
        # de (theta_dot * cos(theta))
        direction_factor = np.sign(theta_dot * np.cos(theta))
        
        u_energy = self.k_energy * E_error * direction_factor
        print(f"u_energy =  {u_energy}")
        
        return u_energy
    
    def compute(self, state):
        """
        Calcule la commande de contrôle par énergie pure.
        
        Args:
            state (array-like): Vecteur d'état [x, x_dot, theta, theta_dot]
            
        Returns:
            float: Commande de contrôle u
        """
        # Extraction des variables d'état
        x, x_dot, theta, theta_dot = state
        
        # Calcul de la commande par énergie uniquement
        u_energy = self._compute_energy_command(theta, theta_dot)
        
        # Sauvegarde pour le débogage
        current_energy = self._compute_energy(theta, theta_dot)
        self.energy_history.append(current_energy)
        
        # Mode dominant - ÉNERGIE UNIQUEMENT
        self.last_mode = "Energy"
        
        return u_energy
    
    def get_info(self):
        """
        Retourne des informations sur l'état du contrôleur (pour débogage).
        
        Returns:
            dict: Informations sur le contrôleur
        """
        return {
            'last_mode': self.last_mode,
            'energy_target': self.E_des,
            # 'theta_threshold': self.theta_threshold,  # DÉSACTIVÉ
            'k_energy': self.k_energy,
            # 'K_lqr': self.K_lqr.tolist(),  # DÉSACTIVÉ
            # 'smooth_transition': self.smooth_transition  # DÉSACTIVÉ
        }