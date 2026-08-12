import os

import numpy as np
import matplotlib.pyplot as plt


class FuzzyController:
    # Classes de sortie (commande moteur) : Gauche Vite, Gauche Doucement, Repos,
    # Droite Doucement, Droite Vite
    OUTPUT_LABELS = ['GV', 'GD', 'R', 'DD', 'DV']

    # Table de règles (carnet) :
    #   lignes   = vitesse angulaire [TED, TD, E, TG, TEG]
    #   colonnes = angle             [EG,  G,  C, D,  ED ]
    #   valeurs  = indice de la classe de sortie dans OUTPUT_LABELS
    #   ⇐⇐=GV(0), ←=GD(1), |=R(2), →=DD(3), ⇒=DV(4)
    RULE_TABLE = np.array([
        # EG  G  C  D  ED
        [4,  3, 1, 0, 0],   # TED : tombe extreme droite
        [4,  3, 2, 1, 0],   # TD  : tombe droite
        [4,  3, 2, 1, 0],   # E   : equilibre -> reaction sur l'angle seul (supprime la zone morte)
        [4,  3, 2, 1, 0],   # TG  : tombe gauche 
        [4,  4, 3, 1, 0],   # TEG : tombe extreme gauche
    ])

    def __init__(self, state, output_centers=(-1.0, -0.3, 0.0, 0.3, 1.0),
                 input_gains=(1.0, 1.0), verbose=False):
        # state : [angle (deg), vitesse angulaire (rad/s)]
        # output_centers : centres de gravité des classes de sortie [GV, GD, R, DD, DV]
        #                  exprimés en RAPPORT CYCLIQUE PWM (bornés à ±1)
        #                  pour la boucle interne, en radians pour la boucle externe
        # input_gains : gains appliqués à [angle, vitesse] avant fuzzification
        #               (paramètres réglables par l'optimiseur : élargir/rétrécir
        #                l'univers de discours sans toucher aux fonctions d'appartenance)
        self.state = state
        self.output_centers = np.asarray(output_centers, dtype=float)
        self.input_gains = np.asarray(input_gains, dtype=float)
        self.verbose = verbose

    def compute(self, state):
        # Interface pull-and-plug (comme LQR.compute / PID.compute)
        self.state = state
        return self.compute_control()

    def compute_control(self):
        scaled_state = [self.state[0] * self.input_gains[0],
                        self.state[1] * self.input_gains[1]]
        fuzzy_value = self.fuzzify(scaled_state)
        inferred_value = self.inference(fuzzy_value)
        control_signal = self.defuzzify(inferred_value)
        return control_signal
    
    def fuzzify(self, input_value):
        # input_value : [angle, angular velocity]
        # Membership function of the angle
        def angle_membership_function(angle):
            fuzzy_angle = np.zeros(5) # extreme gauche, gauche, centre, droite, extreme droite
            if angle < -20:
                fuzzy_angle[0] = 1.0
            elif angle >= -20 and angle <= -10:
                fuzzy_angle[0] = -0.1 * angle - 1.0  # descend de 1 (à -20) vers 0 (à -10)
            else:
                fuzzy_angle[0] = 0.0
            
            if angle >= -20 and angle <= -10:
                fuzzy_angle[1] = 0.1 * angle + 2.0
            elif angle >= -10 and angle <= 0:
                fuzzy_angle[1] = -0.1 * angle
            else:
                fuzzy_angle[1] = 0.0
            
            if angle >= -3 and angle <= 0:
                fuzzy_angle[2] = (1/3) * angle + 1.0
            elif angle >= 0 and angle <= 3:
                fuzzy_angle[2] = -(1/3) * angle + 1.0
            else:
                fuzzy_angle[2] = 0.0
            
            if angle >= 0 and angle <= 10:
                fuzzy_angle[3] = 0.1 * angle
            elif angle >= 10 and angle <= 20:
                fuzzy_angle[3] = -0.1 * angle + 2.0
            else:
                fuzzy_angle[3] = 0.0

            if angle >= 10 and angle <= 20:
                fuzzy_angle[4] = 0.1 * angle - 1.0
            elif angle > 20:
                fuzzy_angle[4] = 1.0
            else:
                fuzzy_angle[4] = 0.0

            return fuzzy_angle
        
        # Membership function of the angular velocity
        def angular_velocity_membership_function(angular_velocity):
            fuzzy_angular_velocity = np.zeros(5) # Tombe Extreme droite, Tombe droite, Equilibre, Tombe gauche, Tombe Extreme gauche
            if angular_velocity < -3:
                fuzzy_angular_velocity[0] = 1.0
            elif angular_velocity >= -3 and angular_velocity <= -1.5:
                fuzzy_angular_velocity[0] = -(2/3) * angular_velocity - 1.0  # descend de 1 (à -3) vers 0 (à -1.5)
            else:
                fuzzy_angular_velocity[0] = 0.0
            
            if angular_velocity >= -3 and angular_velocity <= -1.5:
                fuzzy_angular_velocity[1] = (2/3) * angular_velocity + 2.0
            elif angular_velocity >= -1.5 and angular_velocity <= 0:
                fuzzy_angular_velocity[1] = -(2/3) * angular_velocity
            else:
                fuzzy_angular_velocity[1] = 0.0

            if angular_velocity >= -0.5 and angular_velocity <= 0:
                fuzzy_angular_velocity[2] = (2) * angular_velocity + 1.0
            elif angular_velocity >= 0 and angular_velocity <= 0.5:
                fuzzy_angular_velocity[2] = -(2) * angular_velocity + 1.0
            else:
                fuzzy_angular_velocity[2] = 0.0

            if angular_velocity >= 0 and angular_velocity <= 1.5:
                fuzzy_angular_velocity[3] = (2/3) * angular_velocity
            elif angular_velocity >= 1.5 and angular_velocity <= 3:
                fuzzy_angular_velocity[3] = -(2/3) * angular_velocity + 2.0
            else:
                fuzzy_angular_velocity[3] = 0.0
            
            if angular_velocity >= 1.5 and angular_velocity <= 3:
                fuzzy_angular_velocity[4] = (2/3) * angular_velocity - 1.0
            elif angular_velocity > 3:
                fuzzy_angular_velocity[4] = 1.0
            else:
                fuzzy_angular_velocity[4] = 0.0

            return fuzzy_angular_velocity
        
        def plot_membership_functions(input_value):
            # plot the membership functions for angle and angular velocity and show the fuzzy values for the input value

            angle_range = np.linspace(-30, 30, 100)
            angular_velocity_range = np.linspace(-5, 5, 100)

            angle_membership = np.array([angle_membership_function(angle) for angle in angle_range])
            angular_velocity_membership = np.array([angular_velocity_membership_function(angular_velocity) for angular_velocity in angular_velocity_range])

            plt.figure(figsize=(12, 6))
            plt.subplot(1, 2, 1)
            plt.plot(angle_range, angle_membership)
            # plot the fuzzy values for the input value
            plt.plot(input_value[0], angle_membership_function(input_value[0]), 'ro')
            plt.title('Angle Membership Functions')
            plt.xlabel('Angle (degrees)')
            plt.ylabel('Membership Degree')
            plt.legend(['Extreme Gauche', 'Gauche', 'Centre', 'Droite', 'Extreme Droite'])

            plt.subplot(1, 2, 2)
            plt.plot(angular_velocity_range, angular_velocity_membership)
            # plot the fuzzy values for the input value
            plt.plot(input_value[1], angular_velocity_membership_function(input_value[1]), 'ro')
            plt.title('Angular Velocity Membership Functions')
            plt.xlabel('Angular Velocity (rad/s)')
            plt.ylabel('Membership Degree')
            plt.legend(['Tombe Extreme Droite', 'Tombe Droite', 'Equilibre', 'Tombe Gauche', 'Tombe Extreme Gauche'])

            plt.tight_layout()
            plt.show()

        if self.verbose:
            plot_membership_functions(input_value)
        fuzzy_value = np.zeros((2, 5))
        fuzzy_value[0] = angle_membership_function(input_value[0])
        fuzzy_value[1] = angular_velocity_membership_function(input_value[1])
        if self.verbose:
            print("Fuzzy values for angle and angular velocity:", fuzzy_value)
        return fuzzy_value

    def inference(self, fuzzy_value):
        # Inférence de Mamdani : ET = min, agrégation des règles = max.
        # fuzzy_value[0] = degrés d'appartenance de l'angle   [EG, G, C, D, ED]
        # fuzzy_value[1] = degrés d'appartenance de la vitesse [TED, TD, E, TG, TEG]
        # Retourne le degré d'activation de chaque classe de sortie [GV, GD, R, DD, DV]
        mu_angle, mu_velocity = fuzzy_value
        activation = np.zeros(len(self.OUTPUT_LABELS))
        for i in range(5):        # vitesse angulaire
            for j in range(5):    # angle
                strength = min(mu_angle[j], mu_velocity[i])
                out_class = self.RULE_TABLE[i, j]
                activation[out_class] = max(activation[out_class], strength)
        if self.verbose:
            print("Activations des sorties", dict(zip(self.OUTPUT_LABELS, activation.round(3))))
        return activation

    def defuzzify(self, inferred_value):
        # Défuzzification par centre de gravité (moyenne des centres des classes
        # de sortie pondérée par leur degré d'activation).
        total = np.sum(inferred_value)
        if total == 0.0:
            return 0.0
        return float(np.dot(inferred_value, self.output_centers) / total)

    def export_pipeline_figures(self, input_value=None, save_dir='Figures', prefix='fuzzy'):
        # Exporte en PNG chaque étape de la logique floue pour un état donné :
        #   1. fuzzification (fonctions d'appartenance + point d'entrée)
        #   2. inférence (force de chaque règle + activation des classes de sortie)
        #   3. défuzzification (centres de gravité pondérés + commande finale)
        #   4. surface de commande complète (lookup table)
        # input_value : [angle (deg), vitesse (rad/s)] déjà mis à l'échelle ;
        #               par défaut l'état courant du contrôleur (avec gains).
        os.makedirs(save_dir, exist_ok=True)

        # raw_value = entrées brutes (capteurs) ; input_value = après gains,
        # c'est-à-dire ce que voient réellement les fonctions d'appartenance.
        raw_value = list(self.state)
        if input_value is None:
            input_value = [raw_value[0] * self.input_gains[0],
                           raw_value[1] * self.input_gains[1]]
        else:
            raw_value = [input_value[0] / self.input_gains[0],
                         input_value[1] / self.input_gains[1]]

        verbose_backup = self.verbose
        self.verbose = False
        fuzzy_value = self.fuzzify(input_value)
        activation = self.inference(fuzzy_value)
        u = self.defuzzify(activation)
        mu_angle, mu_velocity = fuzzy_value

        angle_labels = ['EG', 'G', 'C', 'D', 'ED']
        velocity_labels = ['TED', 'TD', 'E', 'TG', 'TEG']

        # --- 1. Fuzzification ---
        angle_range = np.linspace(-30, 30, 200)
        velocity_range = np.linspace(-5, 5, 200)
        angle_curves = np.array([self.fuzzify([a, 0.0])[0] for a in angle_range])
        velocity_curves = np.array([self.fuzzify([0.0, v])[1] for v in velocity_range])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.plot(angle_range, angle_curves)
        ax1.plot([input_value[0]] * 5, mu_angle, 'ro')
        ax1.axvline(input_value[0], color='r', linestyle=':', alpha=0.5)
        ax1.set_title(f'Fuzzification angle = {input_value[0]:.2f} deg')
        ax1.set_xlabel('Angle (deg)')
        ax1.set_ylabel("Degré d'appartenance")
        ax1.legend(angle_labels)
        ax1.grid(alpha=0.3)
        ax2.plot(velocity_range, velocity_curves)
        ax2.plot([input_value[1]] * 5, mu_velocity, 'ro')
        ax2.axvline(input_value[1], color='r', linestyle=':', alpha=0.5)
        ax2.set_title(f'Fuzzification vitesse = {input_value[1]:.2f} rad/s')
        ax2.set_xlabel('Vitesse angulaire (rad/s)')
        ax2.legend(velocity_labels)
        ax2.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(save_dir, f'{prefix}_1_fuzzification.png'), dpi=120)
        plt.close(fig)

        # --- 2. Inférence ---
        rule_strength = np.minimum.outer(mu_velocity, mu_angle)  # min(mu_v[i], mu_a[j])
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        im = ax1.imshow(rule_strength, cmap='YlOrRd', vmin=0, vmax=1)
        ax1.set_xticks(range(5), angle_labels)
        ax1.set_yticks(range(5), velocity_labels)
        ax1.set_xlabel('Angle')
        ax1.set_ylabel('Vitesse angulaire')
        ax1.set_title('Force des règles : min(mu_angle, mu_vitesse)')
        for i in range(5):
            for j in range(5):
                ax1.text(j, i, f'{self.OUTPUT_LABELS[self.RULE_TABLE[i, j]]}\n{rule_strength[i, j]:.2f}',
                         ha='center', va='center', fontsize=8)
        fig.colorbar(im, ax=ax1, fraction=0.046)
        ax2.bar(self.OUTPUT_LABELS, activation, color='steelblue')
        ax2.set_ylim(0, 1)
        ax2.set_title('Activation des classes de sortie (max des règles)')
        ax2.set_ylabel("Degré d'activation")
        ax2.grid(axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(save_dir, f'{prefix}_2_inference.png'), dpi=120)
        plt.close(fig)

        # --- 3. Défuzzification ---
        # Les classes de sortie sont dessinées en triangles (base = centres voisins),
        # découpés au niveau d'activation de chaque règle (Mamdani). Le calcul du
        # contrôleur utilise la version "singletons" : moyenne des centres pondérée
        # par les activations. Les deux centroïdes sont tracés pour comparaison.
        centers = self.output_centers
        widths_left = np.diff(centers, prepend=centers[0] - (centers[1] - centers[0]))
        widths_right = np.diff(centers, append=centers[-1] + (centers[-1] - centers[-2]))
        xs = np.linspace(centers[0] - widths_left[0], centers[-1] + widths_right[-1], 600)

        fig, ax = plt.subplots(figsize=(9, 5))
        aggregated = np.zeros_like(xs)
        for k, (c, a, lab) in enumerate(zip(centers, activation, self.OUTPUT_LABELS)):
            left = np.clip((xs - (c - widths_left[k])) / widths_left[k], 0, None)
            right = np.clip(((c + widths_right[k]) - xs) / widths_right[k], 0, None)
            triangle = np.clip(np.minimum(left, right), 0, 1)
            clipped = np.minimum(triangle, a)          # découpe au niveau d'activation
            aggregated = np.maximum(aggregated, clipped)
            ax.plot(xs, triangle, '--', alpha=0.4)
            ax.annotate(f'{lab}\n(a={a:.2f})', (c, 1.0), textcoords='offset points',
                        xytext=(0, 6), ha='center', fontsize=8)
        ax.fill_between(xs, aggregated, alpha=0.4, color='steelblue',
                        label='Surface agrégée (max des classes découpées)')
        ax.axvline(u, color='g', linewidth=2,
                   label=f'u = {u:.2f} (centres de gravité pondérés, utilisé)')
        if aggregated.sum() > 0:
            centroid = float(np.trapz(xs * aggregated, xs) / np.trapz(aggregated, xs))
            ax.axvline(centroid, color='orange', linestyle=':',
                       label=f'u = {centroid:.2f} (centroïde de la surface, Mamdani)')
        ax.set_xlabel('Commande u')
        ax.set_ylabel("Degré d'appartenance")
        ax.set_title('Défuzzification par centre de gravité')
        ax.set_ylim(0, 1.2)
        ax.legend(loc='lower right', fontsize=8)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(save_dir, f'{prefix}_3_defuzzification.png'), dpi=120)
        plt.close(fig)

        # --- 4. Surface de commande (indexée par les entrées brutes) ---
        angles, velocities, table = self.extract_lookup_table()
        fig, ax = plt.subplots(figsize=(8, 6))
        pcm = ax.pcolormesh(angles, velocities, table, cmap='RdBu_r', shading='auto')
        ax.plot(np.clip(raw_value[0], angles[0], angles[-1]),
                np.clip(raw_value[1], velocities[0], velocities[-1]), 'k*',
                markersize=14, label='état courant')
        fig.colorbar(pcm, ax=ax, label='Commande u')
        ax.set_xlabel('Angle (deg)')
        ax.set_ylabel('Vitesse angulaire (rad/s)')
        ax.set_title('Surface de commande du contrôleur flou')
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(save_dir, f'{prefix}_4_surface_commande.png'), dpi=120)
        plt.close(fig)

        self.verbose = verbose_backup
        print(f">>> Figures du pipeline flou exportées dans '{save_dir}/' (prefixe '{prefix}_')")
        return u

    def extract_lookup_table(self, angle_range=(-30.0, 30.0), velocity_range=(-5.0, 5.0),
                             n_angle=61, n_velocity=41, save_path=None, plot=False,
                             labels=('Angle (deg)', 'Vitesse angulaire (rad/s)', 'Commande u')):
        # Pré-calcule la commande sur une grille (entrée 1 × entrée 2) pour remplacer
        # le pipeline flou par une simple interpolation en temps réel.
        #
        # IMPORTANT : la grille est indexée par les entrées BRUTES (celles que
        # fournissent les capteurs), car compute_control() applique lui-même les
        # input_gains. La table est donc directement exploitable côté embarqué
        # sans avoir à reproduire la mise à l'échelle.
        angles = np.linspace(angle_range[0], angle_range[1], n_angle)
        velocities = np.linspace(velocity_range[0], velocity_range[1], n_velocity)
        table = np.zeros((n_velocity, n_angle))

        verbose_backup = self.verbose
        self.verbose = False
        for i, velocity in enumerate(velocities):
            for j, angle in enumerate(angles):
                table[i, j] = self.compute([angle, velocity])
        self.verbose = verbose_backup

        if save_path is not None:
            # 1ère ligne = entrée 1 (angles), 1ère colonne = entrée 2 (vitesses)
            full = np.zeros((n_velocity + 1, n_angle + 1))
            full[0, 1:] = angles
            full[1:, 0] = velocities
            full[1:, 1:] = table
            np.savetxt(save_path, full, delimiter=',', fmt='%.6f')
            print(f">>> Lookup table sauvegardée dans '{save_path}' "
                  f"({n_velocity}x{n_angle})")

        if plot:
            plt.figure(figsize=(8, 6))
            plt.pcolormesh(angles, velocities, table, cmap='RdBu_r', shading='auto')
            plt.colorbar(label=labels[2])
            plt.xlabel(labels[0])
            plt.ylabel(labels[1])
            plt.title('Lookup table du contrôleur flou')
            plt.show()

        return angles, velocities, table

    @staticmethod
    def load_lookup_table(path):
        # Relit une table sauvegardée par extract_lookup_table().
        # Retourne (entree1, entree2, table) exploitables par lookup_control().
        full = np.loadtxt(path, delimiter=',')
        return full[0, 1:], full[1:, 0], full[1:, 1:]

    @staticmethod
    def lookup_control(angle, velocity, angles, velocities, table):
        # Interpolation bilinéaire dans la lookup table (utilisable sur
        # microcontrôleur sans refaire fuzzification/inférence).
        angle = np.clip(angle, angles[0], angles[-1])
        velocity = np.clip(velocity, velocities[0], velocities[-1])

        j = np.clip(np.searchsorted(angles, angle) - 1, 0, len(angles) - 2)
        i = np.clip(np.searchsorted(velocities, velocity) - 1, 0, len(velocities) - 2)

        ta = (angle - angles[j]) / (angles[j + 1] - angles[j])
        tv = (velocity - velocities[i]) / (velocities[i + 1] - velocities[i])

        low = table[i, j] * (1 - ta) + table[i, j + 1] * ta
        high = table[i + 1, j] * (1 - ta) + table[i + 1, j + 1] * ta
        return float(low * (1 - tv) + high * tv)


def save_cascade_lookup_tables(inner_controller, outer_controller=None,
                               save_dir=os.path.join('Ressources', 'LookupTables'),
                               prefix='fuzzy'):
    """
    Sauvegarde les lookup tables de la cascade floue (CSV + métadonnées).

    Les tables sont indexées par les entrées BRUTES des capteurs :
      - boucle interne : [angle (deg), vitesse angulaire (rad/s)] -> commande u
      - boucle externe : [position x (m), vitesse dx (m/s)]       -> angle cible (rad)

    Format CSV : 1ère ligne = valeurs de l'entrée 1, 1ère colonne = valeurs de
    l'entrée 2, le reste = commande. Relecture via FuzzyController.load_lookup_table().
    """
    os.makedirs(save_dir, exist_ok=True)
    chemins = {}

    # --- Boucle interne (angle -> commande moteur) ---
    chemin = os.path.join(save_dir, f'{prefix}_lookup_angle.csv')
    inner_controller.extract_lookup_table(
        angle_range=(-30.0, 30.0), velocity_range=(-5.0, 5.0),
        n_angle=61, n_velocity=41, save_path=chemin
    )
    chemins['angle'] = chemin

    # --- Boucle externe (position -> angle cible) ---
    if outer_controller is not None:
        # L'univers du fuzzifier (±30 et ±5) rapporté aux unités physiques via
        # les gains : on couvre exactement la plage utile de chaque entrée.
        x_max = 30.0 / outer_controller.input_gains[0]
        dx_max = 5.0 / outer_controller.input_gains[1]
        chemin = os.path.join(save_dir, f'{prefix}_lookup_position.csv')
        outer_controller.extract_lookup_table(
            angle_range=(-x_max, x_max), velocity_range=(-dx_max, dx_max),
            n_angle=61, n_velocity=41, save_path=chemin,
            labels=('Position x (m)', 'Vitesse dx (m/s)', 'Angle cible (rad)')
        )
        chemins['position'] = chemin

    # --- Métadonnées (paramètres ayant généré les tables) ---
    chemin_meta = os.path.join(save_dir, f'{prefix}_lookup_metadata.txt')
    with open(chemin_meta, 'w', encoding='utf-8') as f:
        f.write("Lookup tables de la cascade floue\n")
        f.write("=" * 55 + "\n\n")
        f.write("Format CSV : ligne 0 = entree 1, colonne 0 = entree 2,\n")
        f.write("             le reste = sortie. Case [0,0] inutilisee.\n")
        f.write("Interpolation : FuzzyController.lookup_control()\n")
        f.write("Relecture     : FuzzyController.load_lookup_table()\n\n")
        f.write("Les tables sont indexees par les entrees BRUTES des capteurs\n")
        f.write("(les input_gains sont deja appliques dans les valeurs stockees).\n\n")
        f.write("-- Boucle interne : [angle (deg), vitesse (rad/s)] -> commande u\n")
        f.write(f"   input_gains    = {np.round(inner_controller.input_gains, 6)}\n")
        f.write(f"   output_centers = {np.round(inner_controller.output_centers, 6)}\n")
        if outer_controller is not None:
            f.write("\n-- Boucle externe : [x (m), dx (m/s)] -> angle cible (rad)\n")
            f.write(f"   input_gains    = {np.round(outer_controller.input_gains, 6)}\n")
            f.write(f"   output_centers = {np.round(outer_controller.output_centers, 6)}\n")
        f.write("\n-- Table de regles (lignes = vitesse, colonnes = entree 1)\n")
        for nom, ligne in zip(['TED', 'TD', 'E', 'TG', 'TEG'], FuzzyController.RULE_TABLE):
            etiquettes = ' '.join(f'{FuzzyController.OUTPUT_LABELS[v]:>3}' for v in ligne)
            f.write(f"   {nom:>4} : {etiquettes}\n")
    chemins['metadata'] = chemin_meta
    print(f">>> Métadonnées des lookup tables écrites dans '{chemin_meta}'")

    return chemins


if __name__ == "__main__":
    # Petit test : robot penché à droite (10 deg) et qui tombe à droite (-1 rad/s)
    controller = FuzzyController(state=[10.0, -1.0], verbose=True)
    u = controller.compute_control()
    print(f"Commande floue : u = {u:.3f}")

    angles, velocities, table = controller.extract_lookup_table(plot=True)
    u_lut = FuzzyController.lookup_control(10.0, -1.0, angles, velocities, table)
    print(f"Commande via lookup table : u = {u_lut:.3f}")
