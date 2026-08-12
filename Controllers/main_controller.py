"""
1. créer le contrôleur demandé (et lancer son optimiseur si on le souhaite)
2. dérouler la boucle de simulation pas par pas
3. enregistrer l'historique
4. tracer les courbes et lancer l'animation

À COMPLÉTER : voir les TODO.
"""

import numpy as np
import matplotlib.pyplot as plt

import config
from robot import Robot
from visualizer import Visualizer

from .dummy_controller import DummyController
from .pid_controller import PIDCascade
from .lqr_controller import LQR
from .fuzzy_controller import FuzzyController, save_cascade_lookup_tables
from .sac_controller import SACController        

from Optimizers import pid_optimizer
from Optimizers import lqr_optimizer
from Optimizers import fuzzy_optimizer


class MainController:
    def __init__(self, TYPE_CONTROLEUR="DUMMY", FAIRE_AUTOTUNING=False,
                 FAIRE_VISUALISATION=True, sim_time=20.0,
                 state=np.array([0.0, 0.0, -0.2, 0.0]), target_state=np.zeros(4),
                 save_plot_path=None):
        self.TYPE_CONTROLEUR = TYPE_CONTROLEUR
        self.FAIRE_AUTOTUNING = FAIRE_AUTOTUNING
        self.FAIRE_VISUALISATION = FAIRE_VISUALISATION
        self.sim_time = sim_time
        self.state = state
        self.target_state = target_state
        # Si défini, les courbes sont sauvegardées dans ce fichier au lieu
        # d'être affichées.
        self.save_plot_path = save_plot_path

        self.history = {'time': [], 'theta': [], 'x': [], 'u': [], 'target_theta': []}

    def main(self):
        """Déroule une simulation complète."""

        # =============================================================
        # 1. CRÉATION DU CONTRÔLEUR
        # =============================================================
        # TODO 1 : selon self.TYPE_CONTROLEUR, instancier ce qu'il faut.
    
        pid_cascade = None

        if self.TYPE_CONTROLEUR == "PID":
            # A. Récupération des gains (les DEUX boucles sont réglées ensemble)
            if self.FAIRE_AUTOTUNING:
                print(">>> Auto-Tuning de la CASCADE PID activé.")
                best_gains = pid_optimizer.run_optimization(
                    initial_state=self.state, target_state=self.target_state,
                    sim_time=self.sim_time, save_to_config=True)
            else:
                print(">>> Gains PID chargés depuis config.py")
                best_gains = [config.PID_Kp, config.PID_Ki, config.PID_Kd, config.PID_Pos_Kp, config.PID_Pos_Ki, config.PID_Pos_Kd]

            Kp_angle, Ki_angle, Kd_angle, Kp_pos, Ki_pos, Kd_pos = best_gains

            print(f"Gains PID Angle    -> Kp: {Kp_angle:.4f}, Ki: {Ki_angle:.4f}, Kd: {Kd_angle:.4f}")
            print(f"Gains PID Position -> Kp: {Kp_pos:.4f}, Ki: {Ki_pos:.4f}, Kd: {Kd_pos:.4f}")

            # Cascade complète : la MEME classe est utilisée par l'optimiseur,
            # ce qui garantit que les gains sont évalués sur le système pour
            # lequel ils ont été réglés.
            pid_cascade = PIDCascade(Kp_angle, Ki_angle, Kd_angle, Kp_pos, Ki_pos, Kd_pos, config.dt)
            # Boucle externe (le "Stratège"), floue elle aussi : mêmes règles et
            # fonctions d'appartenance, mais entrées [x (m), dx (m/s)] et sortie
            # = angle cible (rad)
            
            #raise ValueError(f"TYPE_CONTROLEUR '{self.TYPE_CONTROLEUR}' non reconnu. Utilisez 'PID', 'LQR' ou 'FUZZY'")

        if self.TYPE_CONTROLEUR == "FUZZY":
            output_centers = config.FUZZY_OUTPUT_CENTERS
            input_gains = config.FUZZY_INPUT_GAINS
            pos_output_centers = config.FUZZY_POS_OUTPUT_CENTERS
            pos_input_gains = config.FUZZY_POS_INPUT_GAINS

            if self.FAIRE_AUTOTUNING : 
                tuner = fuzzy_optimizer.FuzzyAutoTuner(self.state, self.target_state, self.sim_time)
                result = tuner.optimize( maxiter= 300)
                output_centers, input_gains, pos_output_centers, pos_input_gains = result["output_centers"], result["input_gains"], result["pos_output_centers"], result["pos_input_gains"]
                fuzzy_optimizer.update_config_file(result)
            fuzzy_pos = FuzzyController(self.state, pos_output_centers, pos_input_gains)
            fuzzy_angle = FuzzyController(self.state, output_centers, input_gains)
        #  


        if self.TYPE_CONTROLEUR == "SAC":
            print(">>> Contrôleur SAC activé")

            if self.FAIRE_AUTOTUNING:
                print(">>> Entraînement SAC activé.")
                print(">>> Le modèle entraîné sera sauvegardé dans config.SAC_POLICY_PATH.")
                from Optimizers import sac_trainer
                sac_trainer.main()
                print(">>> Entraînement SAC terminé.")

            print(f"Modèle chargé: {config.SAC_POLICY_PATH}")
            sac_controller = SACController()

        # =============================================================
        # 2. INITIALISATION DE LA SIMULATION
        # =============================================================
        # DONE 2 : créer le Robot, copier l'état initial, calculer le nombre de
        #          pas : steps = int(self.sim_time / config.dt)
        bot = Robot()
        state = self.state
        steps = int(self.sim_time/config.dt)


        # =============================================================
        # 3. BOUCLE TEMPORELLE
        # =============================================================
        # TODO 3 : pour chaque pas i :
        for i in range(steps):
            if self.TYPE_CONTROLEUR == "DUMMY":
                u = dummy_controller.compute(state,self.target_state)
            elif self.TYPE_CONTROLEUR == "PID":
                theta_target = pos_pid_controller.compute(self.target_state[0],self.state[0])
                theta_target = np.clip(theta_target,-0.17,0.17)
                u = -theta_pid_controller.compute(theta_target,state[2])

            else : 
                theta_target = fuzzy_pos.compute([state[0],-state[1]])
                theta_target = np.clip(theta_target,-config.FUZZY_TARGET_THETA_MAX,config.FUZZY_TARGET_THETA_MAX)
                u = -fuzzy_angle.compute([np.degrees(state[2] - theta_target), -state[3]])

            elif self.TYPE_CONTROLEUR == "SAC":
                u = sac_controller.compute(state)
                target_theta = 0.0
                
            x_actuel = state[0]
            theta_actuel = state[2]

            target_theta = self.target_state[2]
            u = 0.0
            if self.TYPE_CONTROLEUR == "PID":
                # --- CASCADE CONTROL (Double Boucle) ---
                # La boucle de position est TOUJOURS active, y compris en
                # auto-tuning : c'est exactement le système que l'optimiseur
                # simule. La désactiver ici (ce que faisait la version
                # précédente) laissait le robot dériver jusqu'à sa vitesse
                # maximale, où les moteurs ne fournissent plus de couple —
                # la chute était alors inévitable.
                u, target_theta = pid_cascade.compute(state,target_x=self.target_state[0])
            # Saturation du rapport cyclique PWM (le pont en H ne peut pas
            # depasser 100 % de la tension batterie)
            u = np.clip(u, -config.PWM_MAX, config.PWM_MAX)

            # Mise à jour Physique
            state = bot.step(state, u, config.dt)

            # Affichage de l'angle à chaque itération
            # print(f"t={i*config.dt:.2f}s | theta={np.degrees(state[2]):7.2f} deg ({state[2]:7.1f} rad) | x={state[0]:6.3f}m | u={u:6.5f}N")

            # Stockage (avant le test de chute, pour garder le dernier point)
            self.history['time'].append(i * config.dt)
            self.history['theta'].append(state[2])
            self.history['x'].append(state[0])
            self.history['u'].append(u)
            self.history['target_theta'].append(self.target_state[2]) 

            self.history['target_theta'].append(target_theta)

            # Arrêt si chute. Le seuil est celui de config.py, le MEME que
            # celui des optimiseurs (l'ancien seuil de pi ne détectait même pas
            # un robot couché à 90°, et l'animation montrait un robot qui
            # tournait sur lui-même).
            if abs(state[2]) > config.CHUTE_THETA_MAX:
                print(f"CRASH (chute) à t={i*config.dt:.2f}s, x={state[0]:.2f} m")
                break
            if abs(state[0]) > config.CHUTE_X_MAX:
                print(f"CRASH (divergence en position) à t={i*config.dt:.2f}s, x={state[0]:.2f} m")
                break

        # On garde le dernier état atteint
              

        #   a) calculer la commande u selon le contrôleur choisi.
        #
        #      DUMMY / LQR / SAC : un seul appel, u = controleur.compute(state, target_state)
        #
        #      PID (cascade) :
        #        1. boucle externe : target_theta = pid_pos.compute(target=0, current=x)
        #           puis SATURER cet angle cible (np.clip a ±0.17 rad, soit 10 deg).
        #           Sans cette sécurité, la boucle externe demande des angles
        #           impossibles et le robot part en avant en accélérant.
        #        2. boucle interne : u = -pid_angle.compute(target=target_theta, current=theta)
        #           Le signe '-' est ESSENTIEL : la commande doit s'opposer à la
        #           chute (u > 0 quand le robot penche vers +theta).
        #
        #      FUZZY (cascade) :
        #        1. target_theta = fuzzy_pos.compute([x, -dx]), saturé à
        #           ±config.FUZZY_TARGET_THETA_MAX
        #        2. u = -fuzzy_angle.compute([degrees(theta - target_theta), -dtheta])
        #           Deux conversions à ne pas oublier : le fuzzifier travaille en
        #           DEGRÉS alors que l'état est en radians, et il attend -dtheta
        #           (voir la convention de signes dans fuzzy_controller.py).
        #
        #   b) saturer la commande : u = np.clip(u, -config.PWM_MAX, config.PWM_MAX).
        #      Le pont en H ne peut pas dépasser 100 % de la tension batterie.
        #
        #   c) faire avancer la physique : state = bot.step(state, u, config.dt)
        #
        #   d) détecter la chute (|theta| trop grand), afficher l'instant du
        #      crash et sortir de la boucle
        #
        #   e) enregistrer time / theta / x / u / target_theta dans self.history

        # DONE 4 : mémoriser le dernier état atteint dans self.state et afficher
        #          un court résumé (angle maximum atteint, par exemple)
        self.state = state
        
        # =============================================================
        # 4. RÉSULTATS
        # =============================================================
        # TODO 5 : appeler self.plot_results() et, si FAIRE_VISUALISATION,
        #          lancer l'animation :
        #               visu = Visualizer(self.history)
        #               visu.animate()
        if self.FAIRE_VISUALISATION :
            visu = Visualizer(self.history)
            visu.animate()
        self.plot_results()


    def plot_results(self, save_path=None):
        """
        Trace les courbes de la simulation.

        Deux graphiques : le suivi d'angle (réel contre cible) et la position au cours du temps.
        """
        if save_path is None:
            save_path = self.save_plot_path

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        ax1.plot(self.history['time'], self.history['theta'], 'b', label="Theta Réel")
        ax1.plot(self.history['time'], self.history['target_theta'], 'r--', alpha=0.5,
                 label="Theta Cible (Demandé)")
        ax1.set_title(f"Suivi d'Angle (Contrôleur : {self.TYPE_CONTROLEUR})")
        ax1.set_ylabel("Theta (rad)")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(self.history['time'], self.history['x'], 'g', label="Position X")
        ax2.axhline(0, color='black', linestyle='--', alpha=0.5)
        ax2.set_ylabel("Position (m)")
        ax2.set_xlabel("Temps (s)")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        if save_path is not None:
            fig.savefig(save_path, dpi=120)
            plt.close(fig)
            print(f">>> Courbes sauvegardées dans '{save_path}'")
        else:
            plt.show()


if __name__ == "__main__":
    controller = MainController()
    controller.main()
