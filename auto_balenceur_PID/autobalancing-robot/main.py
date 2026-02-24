import numpy as np
import matplotlib.pyplot as plt
from robot import Robot
from controller import PID  # Attention au nom du fichier (controller vs controleur)
from controller import EnergyController
import config
import optimizer
from visualizer import Visualizer

# --- PARAMÈTRES ENERGY-LQR (si utilisé) ---
ENERGY_LQR_GAINS = [3.0, 4.0, 50.0, 10.0]  # [K_x, K_x_dot, K_theta, K_theta_dot]
ENERGY_LQR_K_ENERGY = 8.0
ENERGY_LQR_THRESHOLD = 0.25  # rad


# --- CONFIGURATION DU LANCEMENT ---
# TYPE_CONTROLEUR = "PID"  # "PID"
TYPE_CONTROLEUR = "EnergyController"
FAIRE_AUTOTUNING = False  # True = Lance l'optimiseur pour l'angle, False = Simulation normale
FAIRE_VISUALISATION = True  # True = Affiche l'animation

def main():
    # --- 1. INITIALISATION DES CONTROLEURS ---
    Kp_angle, Ki_angle, Kd_angle = 0, 0, 0

    # A. Récupération des gains pour l'ANGLE (Boucle Interne)
    if TYPE_CONTROLEUR == "PID":
        if FAIRE_AUTOTUNING:
            print(">>> Mode : Auto-Tuning de l'ANGLE activé.")
            best_gains = optimizer.run_optimization()
            Kp_angle, Ki_angle, Kd_angle = best_gains
        else:
            print(">>> Mode : Simulation Standard (Gains chargés depuis config.py)")
            Kp_angle = config.PID_Kp
            Ki_angle = config.PID_Ki
            Kd_angle = config.PID_Kd
            
    print(f"Gains Angle utilisés -> Kp: {Kp_angle:.2f}, Kd: {Kd_angle:.2f}")

    # Création du PID Angle (Le "Muscle")
    pid_angle = PID(Kp_angle, Ki_angle, Kd_angle, config.dt)
    
    # Création du PID Position (Le "Stratège")
    pid_pos = PID(config.PID_Pos_Kp, config.PID_Pos_Ki, config.PID_Pos_Kd, config.dt)

    # --- 2. INITIALISATION SIMULATION ---
    bot = Robot()
    
    # État initial :
    # x = 0.0m (Le robot est au point de départ)
    # theta = -0.9 rad (Il démarre légèrement penché)
    state = np.array([-1.0, 0.0, 35 * np.pi/180, 0.0])
    
    # On ajoute 'target_theta' à l'historique pour comprendre ce que veut faire le robot
    history = {'time': [], 'theta': [], 'x': [], 'u': [], 'target_theta': []}
    
    sim_time = 20.0 # On laisse 10 secondes pour qu'il ait le temps de revenir
    steps = int(sim_time / config.dt)

    print(f"Début simulation : Retour au point x=0 depuis x={state[0]}...")

    # --- 3. BOUCLE TEMPORELLE ---
    for i in range(steps):
        x_actuel = state[0]
        theta_actuel = state[2]
        
        target_theta = 0.0
        u = 0.0

        if TYPE_CONTROLEUR == "PID":
            # Si on fait de l'auto-tuning, on ignore la position, on veut juste tenir debout
            if FAIRE_AUTOTUNING:
                target_theta = 0.0
            else:
                # --- CASCADE CONTROL (Double Boucle) ---
                
                # 1. Boucle Externe (Position) : 
                # "Pour aller à 0, quel angle dois-je prendre ?"
                # Cible position = 0
                target_theta = pid_pos.compute(target=0, current=x_actuel)
                
                # SÉCURITÉ : On interdit au robot de demander un angle > 10 degrés (0.17 rad)
                # Sinon il accélère trop fort et tombe.
                target_theta = np.clip(target_theta, -0.17, 0.17)

            # 2. Boucle Interne (Angle) :
            # "J'essaie d'atteindre target_theta calculé au-dessus"
            # Note le signe '-' : u doit s'opposer à la chute
            u = -pid_angle.compute(target=target_theta, current=theta_actuel)
            
        elif TYPE_CONTROLEUR == "EnergyController":
            # Création du contrôleur par énergie (une seule fois au début serait mieux)
            if i == 0:
                print(">>> Contrôleur par Énergie activé")
                print(f"Gain Energy: {ENERGY_LQR_K_ENERGY}")
                energy_controller = EnergyController(
                    m=config.m,
                    M=config.M,
                    l=config.l,
                    g=config.g,
                    dt=config.dt,
                    K_lqr=None,
                    k_energy=ENERGY_LQR_K_ENERGY,
                    theta_threshold=0.2,
                    smooth_transition=False
                )
            u = energy_controller.compute(state)

        # Saturation physique du moteur
        u = np.clip(u, -24, 24)
        
        # Mise à jour Physique
        state = bot.step(state, u, config.dt)
        
        # Arrêt si chute
        if abs(state[2]) > np.pi/3:
            print(f"CRASH à t={i*config.dt:.2f}s")
            #break
            
        # Stockage
        history['time'].append(i * config.dt)
        history['theta'].append(state[2])
        history['x'].append(state[0])
        history['u'].append(u)
        history['target_theta'].append(target_theta)

    # --- 4. AFFICHAGE DES COURBES ---
    # Je ne l'affiche que si on ne fait pas de visualisation, ou alors on ferme pour voir l'anim
    if not FAIRE_VISUALISATION:
        plot_results(history)

    # --- 5. ANIMATION (Dynamique) ---
    if FAIRE_VISUALISATION:
        print("Génération de l'animation...")
        visu = Visualizer(history)
        visu.animate()
        # Optionnel : Afficher les courbes après fermeture de l'animation pour analyse
        plot_results(history)

def plot_results(history):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Graphique 1 : Angles (Réel vs Demandé)
    ax1.plot(history['time'], history['theta'], 'b', label="Theta Réel")
    # On affiche aussi ce que le PID Position demandait (Target)
    ax1.plot(history['time'], history['target_theta'], 'r--', alpha=0.5, label="Theta Cible (Demandé)")
    ax1.set_title(f"Suivi d'Angle (Contrôleur : {TYPE_CONTROLEUR})")
    ax1.set_ylabel("Theta (rad)")
    ax1.legend()
    ax1.grid(True)
    
    # Graphique 2 : Position
    ax2.plot(history['time'], history['x'], 'g', label="Position X")
    ax2.axhline(0, color='black', linestyle='--', alpha=0.5) # La cible x=0
    ax2.set_ylabel("Position (m)")
    ax2.set_xlabel("Temps (s)")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()