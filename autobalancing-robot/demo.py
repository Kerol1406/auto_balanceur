import numpy as np
import time
import sys
from robot import Robot
from controller import PID
from controller import EnergyLQRController
from visualizer import Visualizer
import config

# --- PARAMÈTRES DES SCÉNARIOS ---

def run_simulation(nom_scenario, gains_angle, gains_pos, etat_initial, duree=5.0):
    """
    Fonction générique pour lancer une démo.
    gains_angle : (Kp, Ki, Kd)
    gains_pos   : (Kp, Ki, Kd)
    """
    print(f"\n--- Lancement du Scénario : {nom_scenario} ---")
    print(f"Gains Angle : {gains_angle}")
    print(f"Gains Pos   : {gains_pos}")
    print("Initialisation...")
    time.sleep(1) # Petite pause pour le suspense

    # 1. Configuration
    bot = Robot()
    
    # Contrôleurs forcés avec les paramètres du scénario
    pid_angle = PID(gains_angle[0], gains_angle[1], gains_angle[2], config.dt)
    pid_pos   = PID(gains_pos[0], gains_pos[1], gains_pos[2], config.dt)
    
    state = np.array(etat_initial)
    history = {'time': [], 'theta': [], 'x': [], 'u': [], 'target_theta': []}
    steps = int(duree / config.dt)

    print("Simulation en cours...")

    # 2. Boucle de Simulation
    for i in range(steps):
        x = state[0]
        theta = state[2]
        
        # --- Stratégie de Contrôle (Cascade) ---
        
        # Si les gains de position sont nuls, on désactive cette boucle
        target_theta = 0.0
        if gains_pos[0] != 0 or gains_pos[2] != 0:
            target_theta = pid_pos.compute(target=0.0, current=x)
            target_theta = np.clip(target_theta, -0.2, 0.2) # Max 11 degrés
        
        # Si les gains d'angle sont nuls, u restera à 0 (Scenario 1)
        u = 0.0
        if gains_angle[0] != 0 or gains_angle[2] != 0:
            u = -pid_angle.compute(target=target_theta, current=theta)
        
        # Saturation et Physique
        u = np.clip(u, -24, 24)
        state = bot.step(state, u, config.dt)
        
        # Enregistrement
        history['time'].append(i * config.dt)
        history['theta'].append(state[2])
        history['x'].append(state[0])
        history['u'].append(u)
        history['target_theta'].append(target_theta)
        
        # Gestion du Crash (Arrêt anticipé de la simulation mais on garde l'historique)
        if abs(state[2]) > np.pi/3:
            print(f"💥 CRASH du robot à t = {i*config.dt:.2f}s !")
            # On remplit le reste de l'historique avec la dernière valeur pour que l'anim ne plante pas
            remaining = steps - i - 1
            for _ in range(remaining):
                history['time'].append(history['time'][-1] + config.dt)
                history['theta'].append(state[2]) # Reste au sol
                history['x'].append(state[0])
                history['u'].append(0)
                history['target_theta'].append(target_theta)
            break

    # 3. Lancement de la visualisation
    print("Génération de l'animation... (Fermez la fenêtre pour quitter)")
    visu = Visualizer(history)
    # On surcharge le titre de la fenêtre pour la démo
    visu.animate()

def run_energy_lqr_simulation(nom_scenario, K_lqr, k_energy, theta_threshold, etat_initial, duree=20.0, smooth_transition=True):
    """
    Fonction spéciale pour tester le contrôleur EnergyLQRController.
    K_lqr : [K_x, K_x_dot, K_theta, K_theta_dot] - Gains LQR
    k_energy : Gain pour la commande par énergie
    theta_threshold : Seuil de basculement (rad)
    """
    print(f"\n--- Test EnergyLQRController : {nom_scenario} ---")
    print(f"Gains LQR : {K_lqr}")
    print(f"Gain Energy : {k_energy}")
    print(f"Seuil : {theta_threshold:.2f} rad ({np.degrees(theta_threshold):.1f}°)")
    print(f"Transition douce : {'Oui' if smooth_transition else 'Non'}")
    print("Initialisation...")
    time.sleep(1)

    # 1. Configuration
    bot = Robot()
    
    # Création du nouveau contrôleur hybride
    controller = EnergyLQRController(
        m=config.m,
        M=config.M,
        l=config.l,
        g=config.g,
        dt=config.dt,
        K_lqr=K_lqr,
        k_energy=k_energy,
        theta_threshold=theta_threshold,
        smooth_transition=smooth_transition
    )
    
    state = np.array(etat_initial)
    history = {'time': [], 'theta': [], 'x': [], 'u': [], 'target_theta': [], 'mode': [], 'energy': []}
    steps = int(duree / config.dt)

    print("Simulation en cours...")

    # 2. Boucle de Simulation
    for i in range(steps):
        # Calcul de la commande avec le nouveau contrôleur
        u = controller.compute(state)
        
        # Saturation
        u = np.clip(u, -24, 24)
        
        # Simulation physique
        state = bot.step(state, u, config.dt)
        
        # Informations du contrôleur
        controller_info = controller.get_info()
        current_energy = controller._compute_energy(state[2], state[3])
        
        # Enregistrement étendu
        history['time'].append(i * config.dt)
        history['theta'].append(state[2])
        history['x'].append(state[0])
        history['u'].append(u)
        history['target_theta'].append(0.0)  # Toujours chercher l'équilibre
        history['mode'].append(controller_info['last_mode'])
        history['energy'].append(current_energy)
        
        # Gestion du Crash
        if abs(state[2]) > np.pi/2:  # Plus tolérant car swing-up
            print(f"💥 CRASH du robot à t = {i*config.dt:.2f}s !")
            remaining = steps - i - 1
            for _ in range(remaining):
                history['time'].append(history['time'][-1] + config.dt)
                history['theta'].append(state[2])
                history['x'].append(state[0])
                history['u'].append(0)
                history['target_theta'].append(0.0)
                history['mode'].append('CRASH')
                history['energy'].append(current_energy)
            break

    # 3. Analyse des résultats
    print(f"\n--- Résultats de la Simulation ---")
    print(f"Durée totale : {history['time'][-1]:.2f}s")
    print(f"Angle final : {np.degrees(history['theta'][-1]):.1f}°")
    print(f"Position finale : {history['x'][-1]:.2f}m")
    
    # Statistiques des modes
    modes = history['mode']
    energy_count = modes.count('Energy')
    lqr_count = modes.count('LQR')
    total = len(modes)
    print(f"Temps en mode Energy: {energy_count/total*100:.1f}%")
    print(f"Temps en mode LQR: {lqr_count/total*100:.1f}%")

    # 4. Lancement de la visualisation
    print("Génération de l'animation... (Fermez la fenêtre pour quitter)")
    visu = Visualizer(history)
    visu.animate()

def menu():
    while True:
        print("\n=============================================")
        print("   DÉMO ROBOT AUTO-BALANCEUR - SÉLECTION")
        print("=============================================")
        print("1. Scénario 'Chute Libre' (Pas de contrôle)")
        print("2. Scénario 'Parkinson' (Contrôle inadapté/Instable)")
        print("3. Scénario 'Idéal' (Retour à la base stable)")
        print("4. 🚀 NEW: Test Energy-LQR 'Swing-Up' (Pendule vers le bas)")
        print("5. 🚀 NEW: Test Energy-LQR 'Défi Complet' (Position + Angle extrême)")
        print("6. 🚀 NEW: Test Energy-LQR 'Transition Douce vs Brutale'")
        print("7. Quitter")
        
        choix = input("\nVotre choix (1-7) : ")
        
        if choix == '1':
            # SCÉNARIO 1 : CHUTE LIBRE
            # Conditions : Gains à 0, petit angle initial
            run_simulation(
                nom_scenario="Sans Contrôle (Gravité seule)",
                gains_angle=(0.0, 0.0, 0.0), # Pas de réponse
                gains_pos=(0.0, 0.0, 0.0),
                etat_initial=[0.0, 0.0, 0.1, 0.0], # x=0, theta=0.1 rad
                duree=10.0
            )
            
        elif choix == '2':
            # SCÉNARIO 2 : CONTRÔLE INADAPTÉ
            # Conditions : Kp très fort (réponse violente), Kd nul (pas d'amortissement)
            # Le robot va osciller de plus en plus fort jusqu'au crash
            run_simulation(
                nom_scenario="Contrôle Instable (Oscillations)",
                gains_angle=(3.0, 0.0, 0.05), # Kp trop fort, Kd=0 (Mortel)
                gains_pos=(0.05, 0.0, 0.1),    # Pas de gestion de position
                etat_initial=[0.0, 0.0, 0.5, 0.0], # Petit angle départ
                duree=10.0
            )
            
        elif choix == '3':
            # SCÉNARIO 3 : IDÉAL
            # Conditions : Gains optimaux (ceux que tu as trouvés ou des valeurs sûres)
            # Départ loin (x=-1.5m), doit revenir à 0 en douceur.
            run_simulation(
                nom_scenario="Contrôle Optimal (Retour Base)",
                gains_angle=(2.0, 0.0, 0.1725),   # Tes valeurs PID Angle
                gains_pos=(0.05, 0.0, 0.1),     # Tes valeurs PID Position
                etat_initial=[-1.0, 0.0, 1.0, 0.0], # Départ à -1.5m
                duree=15.0
            )
            
        elif choix == '4':
            # NOUVEAU SCÉNARIO 4 : SWING-UP CLASSIQUE
            # Test du pendule qui part vers le bas (angle = π)
            print("\n Ce test démarre avec le pendule complètement vers le bas.")
            print("Le contrôleur va utiliser la commande par énergie pour faire du swing-up,")
            print("puis basculer vers LQR pour stabiliser à l'équilibre.")
            
            run_energy_lqr_simulation(
                nom_scenario="Swing-Up depuis le bas",
                K_lqr=[2.0, 3.0, 50.0, 10.0],  # Gains LQR optimisés
                k_energy = 3.0,                    # Gain énergie fort
                theta_threshold = 0.3,             
                etat_initial=[0.0, 0.0, 35 * np.pi/180, 0.0], # Pendule vers le bas
                duree = 15.0,
                smooth_transition=True
            )
            
        elif choix == '5':
            # NOUVEAU SCÉNARIO 5 : DÉFI COMPLET
            # Position initiale décalée + angle extrême
            print("\n🔥 Défi EXTREME : Position décalée (-2m) + pendule à 45°")
            print("Le robot doit à la fois revenir au centre ET redresser le pendule.")
            
            run_energy_lqr_simulation(
                nom_scenario="Défi Position + Swing-Up",
                K_lqr=[5.0, 4.0, 60.0, 12.0],        # Gains plus agressifs
                k_energy=12.0,                        # Énergie très forte
                theta_threshold=0.25,                 # Seuil plus strict
                etat_initial=[-2.0, 0.0, np.pi*0.75, 0.0], # -2m, 135°
                duree=25.0,
                smooth_transition=True
            )
            
        elif choix == '6':
            # NOUVEAU SCÉNARIO 6 : COMPARAISON TRANSITIONS
            print("\n⚡ Ce test compare transition DOUCE vs BRUTALE")
            print("Même conditions, mais avec basculement brutal entre modes.")
            
            choix_transition = input("Transition (D)ouce ou (B)rutale ? ").lower()
            smooth = choix_transition == 'd'
            
            run_energy_lqr_simulation(
                nom_scenario=f"Transition {'Douce' if smooth else 'Brutale'}",
                K_lqr=[3.0, 4.0, 45.0, 8.0],
                k_energy=6.0,
                theta_threshold=0.2,
                etat_initial=[0.0, 0.0, np.pi*0.8, 0.0], # 144°
                duree=18.0,
                smooth_transition=smooth
            )
            
        elif choix == '7':
            print("Fin de la démo.")
            sys.exit()
        else:
            print("Choix invalide.")

if __name__ == "__main__":
    try:
        menu()
    except KeyboardInterrupt:
        print("\nArrêt forcé.")