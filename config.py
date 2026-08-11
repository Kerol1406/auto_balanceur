"""
Paramètres du projet, centralisés en un seul endroit.

Tous les autres fichiers font `import config` et lisent leurs constantes ici :
aucune valeur numérique physique ne doit être écrite en dur ailleurs.

À COMPLÉTER : les valeurs marquées `None` doivent être renseignées.
Les paramètres physiques se MESURENT sur le robot réel (balance, réglet) ou se
lisent dans la fiche technique des moteurs. Les gains des contrôleurs, eux,
seront écrits automatiquement dans ce fichier par les optimiseurs.
"""

import numpy as np

# =====================================================================
# 1. PARAMÈTRES PHYSIQUES DU ROBOT
# =====================================================================

# Gravité (m/s^2)
g = 9.81

# --- Masses (kg) ---
# TODO : peser le robot. M = châssis + roues + moteurs (la partie "chariot"),
#        m = tout ce qui se balance au-dessus de l'axe des roues (batterie,
#        cartes, capteurs), m_roue = une seule roue.
m_roue = 0.046      # TODO : masse d'une roue (kg)
M = 0.972           # TODO : masse totale du chariot (kg)
m = 0.39           # TODO : masse du pendule / partie haute (kg) 
M_corps = M-2*m_roue     # TODO : masse du corps sans les deux roues (utile pour l'inertie)

# --- Dimensions (m) ---
d = 0.17    # TODO : distance entre les deux roues (entraxe)
h = 0.075    # TODO : hauteur du corps du robot
b = 0.06  # TODO : demi-longueur du châssis
l = 0.075    # TODO : distance entre l'axe des roues et le centre de gravité du pendule
R = 0.0325    # TODO : rayon des roues

# --- Inerties (kg.m^2) ---
# TODO : écrire les formules, ne pas mettre de nombre "magique".
#   I : moment d'inertie du corps autour de son centre de gravité.
#       Approximation d'une plaque rectangulaire : (1/12) * M_corps * (h^2 + b^2)
#   J : moment d'inertie d'une roue, assimilée à un disque plein : 0.5 * m_roue * R^2
I = (1/12) * M_corps * (h**2 + b**2)    # TODO
J = 0.5 * m_roue * R**2    # TODO

# --- Frottements visqueux ---
bx = 0.1      # Frottement de roulement au sol (N.s/m)
btheta = 0.1  # Frottement dans l'articulation / les réducteurs (N.m.s/rad)

# =====================================================================
# 2. PARAMÈTRES MOTEUR (plateforme YahBoom, moteurs "520" à encodeur)
# =====================================================================
# La commande envoyée au robot est un RAPPORT CYCLIQUE (PWM) dans [-1, 1],
# et NON un couple : le couple réellement disponible dépend aussi de la vitesse
# de rotation (force contre-électromotrice). C'est motor.py qui fait la
# conversion, à partir des grandeurs de la fiche technique ci-dessous.

MOTOR_MODELE = "JGB37-520 12V (1:30)"
MOTOR_COUNT = 2               # Nombre de moteurs (deux roues motrices)
MOTOR_V_ALIM = 11.1          # TODO : tension de la batterie (V)
MOTOR_V_NOM = 12.0           # TODO : tension nominale de la fiche technique (V)
MOTOR_NOLOAD_SPEED_RPM = 333 # TODO : vitesse à vide en sortie de réducteur (tr/min)
MOTOR_NOLOAD_CURRENT = 0.12   # TODO : courant à vide (A)
MOTOR_STALL_TORQUE = 0.4905     # TODO : couple de blocage par moteur (N.m) - attention, les
                              #        fiches donnent des kg.cm : 1 kg.cm = 0.0981 N.m
MOTOR_STALL_CURRENT = 2.3    # TODO : courant de blocage (A)

# Saturation de la commande : rapport cyclique maximal utilisable
PWM_MAX = 1.0

# =====================================================================
# 3. PARAMÈTRES DE SIMULATION
# =====================================================================

dt = 0.005     # Pas de temps de la simulation (s) - doit rester petit devant
               # la dynamique du robot ; c'est aussi la période d'échantillonnage
               # que devra tenir le microcontrôleur.
t_max = 10.0   # Durée par défaut d'une simulation (s)

# =====================================================================
# 4. GAINS DES CONTRÔLEURS
# =====================================================================
# Ces valeurs sont mises à jour AUTOMATIQUEMENT par les fichiers du dossier
# Optimizers/ (fonctions update_config_file). Vous pouvez aussi les régler à la
# main pour comprendre l'effet de chaque terme.

# --- PID : boucle interne (angle) ---
PID_Kp = 40     # TODO : à régler
PID_Ki = 0.0
PID_Kd = 1

# --- PID : boucle externe (position) ---
# Attention : Kp doit rester petit, sinon la boucle externe demande des angles
# cibles énormes et le robot tombe en essayant de les atteindre.
PID_Pos_Kp = 0.05    # TODO
PID_Pos_Ki = 0.0
PID_Pos_Kd = 0.1    # TODO

# --- LQR : matrices de pondération ---
# Q pénalise les écarts d'état [x, dx, theta, dtheta], R pénalise l'effort moteur.
LQR_Q = np.diag([4,1,50,1])   # TODO 
LQR_R = 8                            # TODO

# --- LOGIQUE FLOUE : boucle interne (angle -> commande PWM) ---
# FUZZY_OUTPUT_CENTERS : centres de gravité des classes de sortie
#                        [GV, GD, R, DD, DV], en RAPPORT CYCLIQUE (bornés à ±1)
# FUZZY_INPUT_GAINS    : gains appliqués à [angle (deg), vitesse angulaire (rad/s)]
#                        avant fuzzification (ils dilatent l'univers de discours)
FUZZY_OUTPUT_CENTERS = None   # TODO : np.array de 5 valeurs
FUZZY_INPUT_GAINS = None      # TODO : np.array de 2 valeurs

# --- LOGIQUE FLOUE : boucle externe (position -> angle cible) ---
# Cascade "flou dans flou" : mêmes règles et mêmes fonctions d'appartenance,
# mais les entrées sont [x (m), dx (m/s)] et la sortie est un angle cible (rad).
FUZZY_POS_OUTPUT_CENTERS = None   # TODO : np.array de 5 angles cibles (rad)
FUZZY_POS_INPUT_GAINS = None      # TODO : np.array de 2 gains
FUZZY_TARGET_THETA_MAX = 0.17     # Saturation de l'angle cible (rad), ~10 degrés

# =====================================================================
# 5. APPRENTISSAGE PAR RENFORCEMENT (SAC)
# =====================================================================
# Hyperparamètres du Soft Actor-Critic #TODO.

SAC_POLICY_PATH = "Ressources/sac_policy.pt"  # Où sauvegarder / charger la politique
SAC_CURRICULUM = None          
SAC_HIDDEN_SIZE = None        # Neurones par couche cachée
SAC_LR = None                # Pas d'apprentissage
SAC_GAMMA = None             # Facteur d'actualisation
SAC_TAU = None               # Coefficient de mise à jour douce des réseaux cibles
SAC_BATCH_SIZE = None         # Taille des mini-lots tirés du buffer
SAC_BUFFER_SIZE = None    # Capacité du replay buffer
SAC_TOTAL_STEPS = None    # Nombre de pas d'entraînement
SAC_EPISODE_STEPS = None     # Longueur maximale d'un épisode
SAC_SEED = None                # Graine aléatoire (reproductibilité)
