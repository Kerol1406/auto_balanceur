import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import config


# --- GÉOMÉTRIE DU ROBOT (vue de profil, style YahBoom) ---
# Toutes les pièces sont décrites dans le repère du CORPS :
#   origine = axe des roues, x vers l'arrière/l'avant, y vers le haut.
#   L'avant du robot (capteur ultrason) est du côté des x négatifs.
# Les cotes sont exprimées en mètres et restent cohérentes avec config.py
# (hauteur totale ~ 0.12 m, châssis ~ 0.115 m de long).

def _rect(x0, y0, x1, y1):
    """Rectangle sous forme de tableau de sommets (4x2)."""
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


# (sommets, couleur de remplissage, couleur de bord, ordre d'affichage)
BODY_PARTS = [
    # Bloc moteurs / châssis bas
    (_rect(-0.058, -0.010, 0.058, 0.030), '#3a3a3a', '#101010', 3),
    # Plaque intermédiaire
    (_rect(-0.064, 0.030, 0.064, 0.038), '#141414', '#000000', 4),
    # Boîtier batterie
    (_rect(-0.050, 0.038, 0.050, 0.070), '#2a2a2a', '#101010', 3),
    # Plaque supérieure du bloc batterie
    (_rect(-0.066, 0.070, 0.066, 0.078), '#141414', '#000000', 4),
    # Montants latéraux (jaunes, comme les entretoises du kit)
    (_rect(-0.060, 0.078, -0.054, 0.116), '#d9b310', '#7a6408', 4),
    (_rect(0.054, 0.078, 0.060, 0.116), '#d9b310', '#7a6408', 4),
    # Carte de commande (Arduino / driver moteurs)
    (_rect(0.008, 0.078, 0.048, 0.100), '#1a5fb4', '#0b3268', 5),
    # Module ultrason vu de PROFIL :
    #  - la carte est vue par la tranche (fine bande verticale)
    #  - les deux capsules se superposent : on n'en voit qu'un cylindre,
    #    qui dépasse vers l'avant (x négatif), avec sa grille en bout
    (_rect(-0.014, 0.076, -0.008, 0.110), '#2565c7', '#123a75', 5),   # carte (tranche)
    (_rect(-0.036, 0.083, -0.014, 0.101), '#9aa7b5', '#4a5764', 6),   # corps de la capsule
    (_rect(-0.032, 0.083, -0.030, 0.101), '#7d8a97', 'none', 7),      # nervure du cylindre
    (_rect(-0.038, 0.084, -0.036, 0.100), '#3d4a58', '#2b3947', 7),   # grille avant
    # Plaque de toit
    (_rect(-0.070, 0.116, 0.070, 0.124), '#171717', '#000000', 6),
]


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

    @staticmethod
    def _to_world(points, x_cart, theta):
        """
        Passe du repère du corps au repère monde.
        Le corps tourne de theta autour de l'axe des roues situé en (x_cart, R).
        Un point local (0, l) doit arriver en (x + l.sin(theta), R + l.cos(theta)).
        """
        c, s = np.cos(theta), np.sin(theta)
        rot = np.array([[c, s], [-s, c]])
        return points @ rot.T + np.array([x_cart, config.R])

    def animate(self):
        # 1. Configuration de la fenêtre
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_aspect('equal')  # Important pour que la roue soit ronde !
        ax.grid(True, alpha=0.3)

        # Le robot fait ~12 cm de haut : on cadre serré autour de lui.
        self.window_width = 0.30  # demi-largeur de la fenêtre (m)
        ax.set_ylim(-0.05, 0.31)

        # Titre et Sol
        ax.set_title("Simulation Robot Auto-Balanceur (YahBoom, vue de profil)")
        ax.axhline(0, color='#555555', linewidth=3)  # Le sol

        # --- CRÉATION DES ÉLÉMENTS GRAPHIQUES ---

        # A. Le corps : un ensemble de polygones (châssis, plaques, cartes...)
        self.body_patches = []
        for verts, fc, ec, z in BODY_PARTS:
            p = patches.Polygon(verts, closed=True, fc=fc, ec=ec,
                                linewidth=1.0, zorder=z)
            ax.add_patch(p)
            self.body_patches.append((p, verts))

        # B. La roue : pneu noir + jante bleue + moyeu
        R = config.R
        self.tyre = patches.Circle((0, R), radius=R, fc='#1c1c1c',
                                   ec='#000000', linewidth=1.5, zorder=8)
        self.rim = patches.Circle((0, R), radius=R * 0.68, fc='#2f7fd1',
                                  ec='#12508f', linewidth=1.2, zorder=9)
        self.hub = patches.Circle((0, R), radius=R * 0.18, fc='#c9d3dc',
                                  ec='#5a6672', linewidth=1.0, zorder=11)
        for p in (self.tyre, self.rim, self.hub):
            ax.add_patch(p)

        # C. Les rayons de la jante (pour bien voir la roue tourner)
        self.n_spokes = 6
        self.spokes = [ax.plot([], [], '-', color='#e8f1fa', linewidth=2.0,
                               zorder=10)[0] for _ in range(self.n_spokes)]

        # D. Le centre de masse du pendule (point rouge)
        self.point_com, = ax.plot([], [], 'o', color='#e01b24', markersize=7,
                                  zorder=12, label='Centre de masse')

        # E. Axe du corps (repère visuel de l'inclinaison)
        self.line_axis, = ax.plot([], [], '--', color='#e01b24', linewidth=1.0,
                                  alpha=0.6, zorder=12, label='Axe du corps')

        artists = ([p for p, _ in self.body_patches]
                   + [self.tyre, self.rim, self.hub]
                   + self.spokes + [self.point_com, self.line_axis])

        def init():
            self.point_com.set_data([], [])
            self.line_axis.set_data([], [])
            for s in self.spokes:
                s.set_data([], [])
            return artists

        # Fonction mise à jour à chaque frame "i"
        def update(i):
            current_x = self.x[i]
            current_theta = self.theta[i]

            # 1. Roue : le centre est toujours à (x, R)
            wheel_center = (current_x, config.R)
            self.tyre.center = wheel_center
            self.rim.center = wheel_center
            self.hub.center = wheel_center

            # Angle de rotation de la roue (roulement sans glissement)
            phi = -current_x / config.R
            for k, spoke in enumerate(self.spokes):
                a = phi + 2 * np.pi * k / self.n_spokes
                r0, r1 = config.R * 0.20, config.R * 0.66
                spoke.set_data(
                    [current_x + r0 * np.cos(a), current_x + r1 * np.cos(a)],
                    [config.R + r0 * np.sin(a), config.R + r1 * np.sin(a)],
                )

            # 2. Corps : on incline toutes les pièces autour de l'axe des roues
            for patch, verts in self.body_patches:
                patch.set_xy(self._to_world(verts, current_x, current_theta))

            # 3. Centre de masse et axe du corps
            tip_x = current_x + config.l * np.sin(current_theta)
            tip_y = config.R + config.l * np.cos(current_theta)
            self.point_com.set_data([tip_x], [tip_y])
            self.line_axis.set_data([current_x, tip_x], [config.R, tip_y])

            # --- CAMERA SUIVEUSE ---
            ax.set_xlim(current_x - self.window_width,
                        current_x + self.window_width)

            return artists

        # Création de l'animation
        # frames : combien d'images au total (on peut mettre un 'step' pour accélérer le rendu)
        step_skip = 5  # On affiche 1 image sur 5 pour que ce soit fluide visuellement
        frames_indices = range(0, len(self.t), step_skip)

        ani = animation.FuncAnimation(
            fig,
            update,
            frames=frames_indices,
            init_func=init,
            blit=False,  # Mettre True optimise mais peut bugger sur certains OS
            interval=self.interval * step_skip,
            repeat=False
        )

        ax.legend(loc='upper right')
        plt.show()
