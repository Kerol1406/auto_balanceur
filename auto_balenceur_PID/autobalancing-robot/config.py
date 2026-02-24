# --- PARAMÈTRES PHYSIQUES DU ROBOT ---

# Gravité (m/s^2)
g = 9.81

# Masses (kg)
M = 1.0   # Masse du chariot (roues + moteurs + châssis bas)
m = 0.5   # Masse du pendule (tige + batterie + électronique en haut)

# Dimensions (m)
l = 0.3   # Distance entre l'axe des roues et le centre de gravité du pendule
R = 0.05  # Rayon des roues

# Inerties (kg.m^2)
I = 0.006 # Moment d'inertie du pendule autour de son centre de gravité
J = 0.001 # Moment d'inertie des roues (approximation)

# Frottements (Visqueux)
bx = 0.1      # Frottement au sol (chariot)
btheta = 0.1  # Frottement à l'articulation (pendule)

# --- PARAMÈTRES DE SIMULATION ---
dt = 0.01     # Pas de temps de la simulation (10ms)
t_max = 10.0  # Durée totale de la simulation (secondes)

# --- PARAMÈTRES DE CONTRÔLE (PID) ---
# PID ANGLE (Boucle Interne - Rapide)
# Valeurs trouvées par ton Auto-Tuner (ou valeurs manuelles)
# Ces valeurs sont mises à jour automatiquement par optimizer.py
PID_Kp = 1.9897
PID_Ki = 0.0000
PID_Kd = 0.1725
# PID_Kp = 3
# PID_Ki = 0.0000
# PID_Kd = 0.05

# PID POSITION (Boucle Externe - Lente)
# Attention : KP doit être petit pour ne pas demander des angles de fou
PID_Pos_Kp = 0.05 
PID_Pos_Ki = 0.0
PID_Pos_Kd = 0.1  # Le Kd aide beaucoup pour freiner en arrivant sur la cible
