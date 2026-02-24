import numpy as np
import re # Pour manipuler le texte du fichier config
from robot import Robot
from controller import PID
import config

class AutoTuner:
    def __init__(self):
        self.params = [0.0, 0.0, 0.0] 
        self.dparams = [5.0, 0.0, 1.0] # Kp, Ki, Kd (On ne tune pas Ki)
        self.start_angle = 0.05 # Angle faible pour le tuning

    def run_simulation(self, p):
        kp, ki, kd = p
        bot = Robot()
        pid = PID(kp, ki, kd, config.dt)
        state = np.array([0.0, 0.0, self.start_angle, 0.0])
        
        total_error = 0.0
        steps = int(4.0 / config.dt) # 4 secondes suffisent
        
        for _ in range(steps):
            u = -pid.compute(0, state[2])
            u = np.clip(u, -24, 24)
            state = bot.step(state, u, config.dt)
            
            # Fonction de coût : Angle^2 + Vitesse_Angulaire^2 + Position^2 (pour éviter qu'il parte loin)
            total_error += (state[2]**2) + 0.01*(state[3]**2) + 0.01*(state[0]**2)
            
            if abs(state[2]) > np.pi/3:
                total_error += 1e6 # Pénalité crash
                break 
        return total_error

    def twiddle(self, tol=0.2):
        print(">>> Démarrage de l'auto-tuning...")
        best_err = self.run_simulation(self.params)
        
        while sum(self.dparams) > tol:
            for i in range(len(self.params)):
                if self.dparams[i] == 0: continue # On saute Ki si dparam est 0

                self.params[i] += self.dparams[i]
                err = self.run_simulation(self.params)
                
                if err < best_err:
                    best_err = err
                    self.dparams[i] *= 1.1
                else:
                    self.params[i] -= 2 * self.dparams[i]
                    err = self.run_simulation(self.params)
                    if err < best_err:
                        best_err = err
                        self.dparams[i] *= 1.1
                    else:
                        self.params[i] += self.dparams[i]
                        self.dparams[i] *= 0.9
                        
        print(f">>> Tuning terminé. Meilleurs gains : {self.params}")
        return self.params

def update_config_file(kp, ki, kd):
    """
    Lit config.py, remplace les lignes PID_... et sauvegarde.
    """
    print(">>> Sauvegarde des nouveaux gains dans config.py...")
    
    with open("config.py", "r") as f:
        lines = f.readlines()

    with open("config.py", "w") as f:
        for line in lines:
            # On cherche et remplace les lignes spécifiques
            if line.startswith("PID_Kp"):
                f.write(f"PID_Kp = {kp:.4f}\n")
            elif line.startswith("PID_Ki"):
                f.write(f"PID_Ki = {ki:.4f}\n")
            elif line.startswith("PID_Kd"):
                f.write(f"PID_Kd = {kd:.4f}\n")
            else:
                f.write(line) # On réécrit les autres lignes à l'identique

def run_optimization():
    """Fonction principale appelée par main.py"""
    tuner = AutoTuner()
    # On force Ki à 0 (dparams=0)
    tuner.dparams = [2.0, 0.0, 0.5] 
    
    best_params = tuner.twiddle(tol=0.05)
    
    # Mise à jour du fichier
    update_config_file(best_params[0], best_params[1], best_params[2])
    
    return best_params # On retourne les valeurs pour usage immédiat