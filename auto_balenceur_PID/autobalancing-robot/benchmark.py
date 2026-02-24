import numpy as np
import matplotlib.pyplot as plt
from robot import Robot
from controller import PID
import config
from visualizer import Visualizer

class Benchmark:
    def __init__(self):
        self.results = {}

    def run_scenario(self, scenario_name, setup_func, duration=10.0):
        """
        Exécute un scénario donné et calcule les métriques.
        setup_func : Fonction qui configure le robot et le contrôleur pour ce test.
        """
        print(f"--- Exécution du test : {scenario_name} ---")
        
        # 1. Configuration spécifique au scénario
        bot, pid_angle, pid_pos, state, noise_level, kick_func = setup_func()
        
        history = {'time': [], 'theta': [], 'x': [], 'u': []}
        total_energy = 0.0
        max_overshoot = 0.0
        settling_time = None
        
        # Paramètres de simulation
        duration = duration 
        steps = int(duration / config.dt)
        
        # 2. Boucle de simulation
        for i in range(steps):
            t = i * config.dt
            
            # A. Injection de Bruit (Capteur)
            # Le contrôleur voit une valeur bruitée, mais la physique utilise la vraie valeur
            noise = np.random.normal(0, noise_level) if noise_level > 0 else 0
            measured_theta = state[2] + noise
            measured_x = state[0] # On suppose x moins bruité pour simplifier
            
            # B. Calcul du PID (Double boucle)
            target_theta = pid_pos.compute(0, measured_x)
            target_theta = np.clip(target_theta, -0.2, 0.2)
            u = -pid_angle.compute(target_theta, measured_theta)
            u = np.clip(u, -24, 24)
            
            # C. Injection de Perturbation (Kick)
            f_ext = kick_func(t)
            
            # D. Physique
            state = bot.step(state, u, config.dt, f_ext)
            
            # E. Calcul des Métriques en direct
            total_energy += (u**2) * config.dt # Intégrale de u²
            
            current_abs_theta = abs(state[2])
            if current_abs_theta > max_overshoot:
                max_overshoot = current_abs_theta
                
            # Détection temps de stabilisation (Si on reste sous 2 degrés soit 0.035 rad)
            if current_abs_theta > 0.035:
                settling_time = t # Tant qu'on dépasse, on reset le temps
            
            # Stockage
            history['time'].append(t)
            history['theta'].append(state[2])
            history['x'].append(state[0])
            history['u'].append(u)

        # Si settling_time est égal au temps final, c'est qu'on ne s'est jamais stabilisé
        if settling_time is not None and settling_time > duration - 0.1:
            settling_time = float('inf') # Échec
            
        metrics = {
            "Energy": total_energy,
            "Max Angle (rad)": max_overshoot,
            "Settling Time (s)": settling_time if settling_time else 0.0,
            "Final Position": state[0]
        }

        visu = Visualizer(history)
        visu.animate()
        
        self.results[scenario_name] = (metrics, history)
        return metrics

    def report(self):
        """Affiche un tableau comparatif"""
        print("\n" + "="*60)
        print(f"{'SCÉNARIO':<25} | {'ÉNERGIE (V²s)':<12} | {'MAX ANGLE':<10} | {'STABILISÉ (s)':<12}")
        print("-" * 60)
        for name, (m, _) in self.results.items():
            stab = f"{m['Settling Time (s)']:.2f}" if m['Settling Time (s)'] != float('inf') else "FAIL"
            print(f"{name:<25} | {m['Energy']:<12.1f} | {m['Max Angle (rad)']:<10.3f} | {stab:<12}")
        print("="*60 + "\n")

    def plot_all(self):
        """Trace les courbes de tous les scénarios"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()
        
        for i, (name, (m, h)) in enumerate(self.results.items()):
            ax = axes[i]
            ax.plot(h['time'], h['theta'], label='Angle')
            ax.set_title(f"{name}")
            ax.set_ylim(-0.5, 0.5)
            ax.grid(True)
            ax.legend()
            
        plt.tight_layout()
        plt.show()

# --- DÉFINITION DES SCÉNARIOS ---

def setup_angle_initial():
    bot = Robot() # Masse normale
    pid_angle = PID(config.PID_Kp, 0.0, config.PID_Kd, config.dt)
    pid_pos = PID(0.05, 0.0, 0.1, config.dt)
    # Départ à 20 degrés (0.35 rad)
    state = [0.0, 0.0, 0.35, 0.0]
    return bot, pid_angle, pid_pos, state, 0.0, lambda t: 0

def setup_kick():
    bot = Robot()
    pid_angle = PID(config.PID_Kp, 0.0, config.PID_Kd, config.dt)
    pid_pos = PID(0.05, 0.0, 0.1, config.dt)
    state = [0.0, 0.0, 0.35, 0.0] # Départ stable
    
    # FONCTION KICK : Force de 15 Newtons entre 1.0s et 1.1s
    def kick(t):
        return 15.0 if 1.0 <= t <= 1.1 else 0.0
        
    return bot, pid_angle, pid_pos, state, 0.0, kick

def setup_noise():
    bot = Robot()
    # PID : On baisse un peu le Kd car la dérivée du bruit est violente
    pid_angle = PID(config.PID_Kp, 0.0, config.PID_Kd, config.dt) 
    pid_pos = PID(0.05, 0.0, 0.1, config.dt)
    state = [0.0, 0.0, 0.35, 0.0] # Petit angle
    # BRUIT : Sigma = 0.02 rad
    return bot, pid_angle, pid_pos, state, 0.02, lambda t: 0

def setup_robustness():
    # TEST: Robot 30% plus lourd en haut !
    bot = Robot(mass_factor=1.3) 
    # On garde le MÊME PID qu'avant
    pid_angle = PID(config.PID_Kp, 0.0, config.PID_Kd, config.dt)
    pid_pos = PID(0.05, 0.0, 0.1, config.dt)
    state = [0.0, 0.0, 0.35, 0.0]
    return bot, pid_angle, pid_pos, state, 0.0, lambda t: 0

# --- EXÉCUTION ---
if __name__ == "__main__":
    bench = Benchmark()
    
    # Lancement des 4 épreuves
    bench.run_scenario("1. Angle Initial 20deg", setup_angle_initial)
    bench.run_scenario("2. Perturbation (Kick)", setup_kick)
    bench.run_scenario("3. Bruit Capteur", setup_noise)
    bench.run_scenario("4. Robustesse (+30% Masse)", setup_robustness)
    
    # Résultats
    bench.report()
    bench.plot_all()