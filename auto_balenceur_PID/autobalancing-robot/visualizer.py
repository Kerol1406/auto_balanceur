import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import config

class Visualizer:
    def __init__(self, history):
        """
        history : dictionnaire contenant 'time', 'x', 'theta'
        """
        self.t = history['time']
        self.x = history['x']
        self.theta = history['theta']
        
        # Intervalle entre deux images (en ms)
        # On essaie de coller au temps réel, mais ça dépend de la vitesse du PC
        self.interval = config.dt * 1000 

    def animate(self):
        # 1. Configuration de la fenêtre
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_aspect('equal') # Important pour que la roue soit ronde !
        ax.grid(True)
        ax.set_ylim(-0.5, 1.5) # On regarde de un peu sous le sol jusqu'à 1.5m de haut
        
        # Titre et Sol
        ax.set_title(f"Simulation Robot Auto-Balanceur (Vitesse x{1.0})")
        ax.axhline(0, color='black', linewidth=2) # Le sol
        
        # --- CRÉATION DES ÉLÉMENTS GRAPHIQUES (Vides au début) ---
        
        # A. Le Corps (Une ligne épaisse)
        # On initialise avec des données vides [], []
        line_body, = ax.plot([], [], 'k-', linewidth=4, label='Corps')
        
        # B. La Roue (Un cercle bleu)
        wheel_radius = config.R
        patch_wheel = patches.Circle((0, wheel_radius), radius=wheel_radius, fc='blue', ec='black')
        ax.add_patch(patch_wheel)
        
        # C. Le rayon de la roue (Pour voir qu'elle tourne)
        line_wheel_radius, = ax.plot([], [], 'w-', linewidth=2) # Ligne blanche
        
        # D. Le centre de masse (Un point rouge)
        point_com, = ax.plot([], [], 'ro', markersize=8)

        # Fonction d'initialisation pour l'animation
        def init():
            line_body.set_data([], [])
            line_wheel_radius.set_data([], [])
            point_com.set_data([], [])
            patch_wheel.center = (0, wheel_radius)
            return line_body, patch_wheel, line_wheel_radius, point_com

        # Fonction mise à jour à chaque frame "i"
        def update(i):
            # Récupération de l'état à l'instant i
            current_x = self.x[i]
            current_theta = self.theta[i]
            
            # --- CALCUL GÉOMÉTRIQUE ---
            
            # 1. Position de la roue
            # Le centre est à (x, R)
            wheel_center = (current_x, config.R)
            patch_wheel.center = wheel_center
            
            # 2. Position du corps (Tige)
            # Base = Centre roue
            # Sommet = Base + Longueur * [sin(theta), cos(theta)]
            tip_x = current_x + config.l * np.sin(current_theta)
            tip_y = config.R + config.l * np.cos(current_theta)
            
            line_body.set_data([current_x, tip_x], [config.R, tip_y])
            
            # 3. Position du rayon de la roue (Visualisation rotation)
            # L'angle de rotation de la roue (phi) est lié à x : phi = x / R
            phi = current_x / config.R
            # Un point sur le bord de la roue
            rim_x = current_x + config.R * np.sin(phi)
            rim_y = config.R + config.R * np.cos(phi)
            
            line_wheel_radius.set_data([current_x, rim_x], [config.R, rim_y])
            
            # 4. Centre de masse (Sommet)
            point_com.set_data([tip_x], [tip_y])
            
            # --- CAMERA SUIVEUSE ---
            # On déplace la fenêtre pour que le robot reste au centre
            window_width = 2.0 # Mètres de chaque côté
            ax.set_xlim(current_x - window_width, current_x + window_width)
            
            return line_body, patch_wheel, line_wheel_radius, point_com

        # Création de l'animation
        # frames : combien d'images au total (on peut mettre un 'step' pour accélérer le rendu)
        step_skip = 5 # On affiche 1 image sur 5 pour que ce soit fluide visuellement
        frames_indices = range(0, len(self.t), step_skip)
        
        ani = animation.FuncAnimation(
            fig, 
            update, 
            frames=frames_indices,
            init_func=init,
            blit=False, # Mettre True optimise mais peut bugger sur certains OS
            interval=self.interval * step_skip,
            repeat=False
        )
        
        plt.legend()
        plt.show()