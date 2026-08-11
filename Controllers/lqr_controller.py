import numpy as np
from scipy import linalg
import config
from robot import Robot

class LQR:
    """
    Contrôleur LQR (Linear Quadratic Regulator) pour le pendule inversé.

    Ce contrôleur calcule les gains optimaux K en résolvant l'équation de Riccati
    algébrique pour minimiser le coût quadratique J = ∫(x'Qx + u'Ru)dt

    MODÈLE D'ESPACE D'ÉTAT:
    ----------------------
    Le système est linéarisé autour de theta = 0 (pendule en haut):

        dx/dt = Ax + Bu

    où x = [x, x_dot, theta, theta_dot]^T est le vecteur d'état:
        - x: position du chariot (m)
        - x_dot: vites   cse du chariot (m/s)
        - theta: angle du pendule par rapport à la verticale (rad) 
        - theta_dot: vitesse angulaire (rad/s)

    et u est la commande appliquée (rapport cyclique PWM dans [-1, 1])

    CHOIX DES MATRICES Q et R:
    -------------------------
    Q (4x4): Pénalise les écarts d'état par rapport à l'équilibre
        - Q[0,0]: Pénalité sur la position x -> limiter l'écart horizontal
        - Q[1,1]: Pénalité sur la vitesse x_dot -> amortir les oscillations
        - Q[2,2]: Pénalité sur l'angle theta -> PRIORITAIRE pour stabiliser le pendule
        - Q[3,3]: Pénalité sur theta_dot -> amortir les oscillations angulaires

    Recommandations:
        - Q[2,2] >> autres termes (ex: 100 vs 1) pour privilégier l'angle
        - Q[3,3] modéré (ex: 10) pour l'amortissement angulaire
        - Q[0,0] et Q[1,1] faibles (ex: 1) si position x peu importante

    R (scalaire ou 1x1): Pénalise l'effort moteur
        - R petit -> effort moteur important autorisé (réponse rapide)
        - R grand -> effort moteur limité (économie d'énergie, douceur)
        - Typiquement: R = 1 comme référence, ajuster selon le moteur

    LOI DE COMMANDE:
    ---------------
    u = -K*(x - x_target)

    où K = [K_x, K_x_dot, K_theta, K_theta_dot] sont les gains optimaux calculés
    """

    def __init__(self, Q=None, R=None, verbose=False):
        """
        Initialise le contrôleur LQR.

        Args:
            Q (array-like): Matrice de pondération des états (4x4)
                           Si None, utilise une valeur par défaut (TODO 1)
            R (float or array): Matrice de pondération de la commande (scalaire ou 1x1)
                               Si None, utilise une valeur par défaut (TODO 1)
            verbose (bool): Affiche les informations de calcul
        """

        self.m = config.m
        self.M = config.M
        self.l = config.l
        self.g = config.g
        self.verbose = verbose

        # ============================================================
        # TODO 1 : Valeurs par défaut de Q et R
        # ============================================================
        # Objectif : si l'utilisateur ne fournit pas Q, on doit quand même
        # avoir une matrice de pondération raisonnable pour stabiliser le
        # pendule. Même chose pour R.
        #
        # Rappel (voir docstring de la classe ci-dessus) :
        #   - Q est une matrice diagonale 4x4, pondérant [x, x_dot, theta, theta_dot]
        #   - Q[2,2] (theta) doit dominer largement les autres termes
        #   - R est un scalaire (ou une matrice 1x1), typiquement proche de 1
        #
        # Indice : np.diag([...]) construit une matrice diagonale à partir
        # d'une liste de 4 valeurs.
        #
        # Complètez les deux blocs ci-dessous :

        if Q is None:
            # TODO: remplace cette ligne par une matrice Q par défaut cohérente
            Q = np.diag([1.0, 1.0, 1.0, 1.0])  # <-- à remplacer
        else:
            Q = np.array(Q)

        if R is None:
            # TODO: remplace cette ligne par une valeur R par défaut cohérente
            R = np.array([[1]])  # <-- à remplacer
        else:
            R = np.array([[R]]) if np.isscalar(R) else np.array(R)
        # ============================================================
        # FIN TODO 1
        # ============================================================

        self.Q = Q
        self.R = R

        # Construction des matrices du système linéarisé
        self.A, self.B = self._build_system_matrices()

        # Calcul des gains LQR optimaux
        self.K = self._compute_lqr_gains()
        
        if self.verbose:
            self.print_system_info()

    def _build_system_matrices(self, eps=1e-6):
        """
        Construit les matrices A et B du système linéarisé autour de l'équilibre.

        dx/dt = A*x + B*d     avec d = rapport cyclique PWM dans [-1, 1]

        La linéarisation est faite NUMÉRIQUEMENT (différences finies centrées)
        sur robot.derivarives(). C'est volontaire :

        1. L'entrée du système n'est plus un couple mais un rapport cyclique.
           Le couple délivré vaut K_duty*d - C_bemf*((dx/R) - dtheta) : la force
           contre-électromotrice introduit des termes d'amortissement croisés
           entre x_dot et theta_dot qui alourdissent la dérivation à la main.

        2. Le modèle du LQR est ainsi garanti cohérent avec le simulateur : si
           robot.py change, A et B suivent automatiquement, ce qui supprime tout
           risque de divergence entre le modèle de commande et la physique.

        Returns:
            A (ndarray): Matrice d'état (4x4)
            B (ndarray): Matrice de commande (4x1)
        """
        bot = Robot()
        etat_eq = np.zeros(4)   # équilibre : pendule vertical, robot immobile
        duty_eq = 0.0

        # ============================================================
        # DONE 2 : Linéarisation numérique par différences finies centrées
        # ============================================================
        # Rappel théorique :
        #   A = ∂f/∂état  (jacobienne de la dynamique par rapport à l'état)
        #   B = ∂f/∂commande (jacobienne par rapport à la commande)
        #
        # où f = bot.derivarives(état, commande) donne dx/dt.
        #
        # Pour une différence finie centrée sur la variable j de l'état :
        #   colonne_j(A) ≈ [f(état + eps*e_j, u0) - f(état - eps*e_j, u0)] / (2*eps)
        #
        # où e_j est le vecteur unitaire dans la direction j (une seule
        # composante de l'état perturbée à la fois, les 3 autres à 0).
        #
        # Étapes à coder :
        #   1. Crée A = np.zeros((4, 4))
        #   2. Pour j allant de 0 à 3 :
        #        a. crée un vecteur de perturbation nul, mets eps à l'indice j
        #        b. calcule f_plus = bot.derivarives(etat_eq + perturbation, duty_eq)
        #        c. calcule f_moins = bot.derivarives(etat_eq - perturbation, duty_eq)
        #        d. remplis la colonne j de A : A[:, j] = (f_plus - f_moins) / (2*eps)
        #   3. Fais la même chose pour B, mais en perturbant la commande
        #      (duty_eq + eps) et (duty_eq - eps) au lieu de l'état.
        #      B doit être remis en forme (4, 1) avec .reshape(4, 1)
        #
        # Astuce de vérification : une fois codé, tu dois retrouver
        # exactement A[0,1] = 1.0 et A[2,3] = 1.0 (car dx/dt = x_dot et
        # dtheta/dt = theta_dot, ce qui est vrai par construction du modèle,
        # indépendamment de la physique précise). Si ce n'est pas le cas,
        # il y a une erreur dans la boucle.

        A = np.zeros((4, 4))
        
        for j in range(4):
            perturbation = np.zeros(4)
            perturbation[j] = eps
            f_plus = bot.derivarives(etat_eq + perturbation, duty_eq)
            f_moins = bot.derivarives(etat_eq - perturbation, duty_eq)

            f_plus_B = bot.derivarives(etat_eq, duty_eq + eps)
            f_moins_B = bot.derivarives(etat_eq, duty_eq - eps)
            
            A[:, j] = (f_plus - f_moins) / (2*eps)
        B = (f_plus_B - f_moins_B) / (2*eps)
        B = B.reshape(4, 1)
        # ============================================================
        # FIN TODO 2
        # ============================================================
        return A, B

    def _build_system_matrices1(self):
        """
        [BONUS — optionnel] Construit A et B par dérivation ANALYTIQUE,
        à comparer avec la version numérique de _build_system_matrices().

        Cette méthode n'est PAS appelée par __init__ (le contrôleur utilise
        toujours _build_system_matrices, la version numérique). L'intérêt
        ici est pédagogique : traduire des formules physiques déjà dérivées
        en code, puis vérifier qu'on retombe sur les mêmes valeurs que la
        méthode par différences finies.

        Le système non-linéaire est linéarisé autour de (x=0, theta=0).
        IMPORTANT: ces matrices doivent correspondre au modèle physique de
        robot.py !

        Returns:
            A (ndarray): Matrice d'état (4x4)
            B (ndarray): Matrice de commande (4x1)
        """
        m = self.m
        M = self.M
        l = self.l
        g = self.g

        # Paramètres identiques à robot.py (config.py)
        R_wheel = config.R      # Rayon des roues
        I = config.I            # Moment d'inertie du pendule
        J = config.J            # Moment d'inertie des roues
        bx = config.bx          # Frottement chariot
        btheta = config.btheta  # Frottement pendule
        M_corps = config.M_corps 
        m_roue = config.m_roue

        # ============================================================
        # DONE 5a [BONUS] : masses effectives et dénominateur couplé
        # ============================================================
        # Rappel (modèle couplé chariot/pendule, comme dans robot.py) :
        #   M_total = M + m + J / R_wheel^2
        #   I_total = I + m * l^2
        #   det_M   = M_total * I_total - (m*l)^2
        #
        # Ces trois quantités serviront aux coefficients ci-dessous.

        M_total = M + m + J / R_wheel**2  # <-- à remplacer
        I_total = I + m * l**2  # <-- à remplacer
        det_M   = M_total * I_total - (m*l)**2    # <-- à remplacer

        # ============================================================
        # FIN TODO 5a
        # ============================================================

        # ============================================================
        # DONE 5b : coefficients de A et B (modèle "chariot-pendule")
        # ============================================================
        # Rappel des formules à traduire en code (ce sont exactement les
        # coefficients qu'on avait dérivés analytiquement pour le modèle
        # à couple/force en entrée) :
        # Indice : np.array([[...], [...], [...], [...]]) pour une matrice
        # 4x4 ; B_v1 doit avoir la forme (4, 1).
        A11, A12, A13, A14 = 0, 1/det_M, 0, 0
        A21, A22, A23, A24 = 0, -I_total*bx, (m**2)*(l**2)*g, 0
        A31, A32, A33, A34 = 0, 0, 0, 1/det_M
        A41, A42, A43, A44 = 0, -m*l*bx, M_total*m*g*l, 0

        A_v1 = 1/det_M*np.array([[A11, A12, A13, A14],
                                 [A21, A22, A23, A24],
                                 [A31, A32, A33, A34],
                                 [A41, A42, A43, A44]])  # <-- à remplacer (matrice 4x4)

        B2 = I_total/(2*R_wheel)-m*l
        B3 = m*l/(2*R_wheel) - M_total
        B_v1 =1/det_M*np.array([[0],[B2],[B3],[0]])  # <-- à remplacer (matrice 4x1)
        
    
        # ============================================================
        # FIN TODO 5b
        # ============================================================

        # ============================================================
        # TODO 5c [BONUS] : coefficients de A et B (modèle "robot à roues",
        # Facultatif (Modèle implémenté en MATLAB par Yahboom)
        # ============================================================
        # Ce deuxième jeu de formules correspond à la paramétrisation
        # "robot à deux roues" (avec masse de roue m_roue et masse de corps
        # M_corps séparées). Formules à traduire :
        #
        #   Q_denom = J*M + (J + M_corps*l^2) * (2*m_roue + 2*I/R_wheel^2)
        #
        #   A_23 = -(M^2 * l^2 * g) / Q_denom
        #   A_43 = M*l*g * (M + 2*m_roue + 2*I/R_wheel^2) / Q_denom
        #
        #   B_21 = (J + M*l^2 + M*l*R_wheel) / (Q_denom * R_wheel)
        #   B_41 = -(M*l/R_wheel + M + 2*m_roue + 2*I/R_wheel^2) / Q_denom
        #
        # Puis construis (attention à l'échelle par I/R_wheel sur B) :
        #
        #   A_v2 = [[0, 1,    0,    0],
        #           [0, 0,  A_23,   0],
        #           [0, 0,    0,    1],
        #           [0, 0,  A_43,   0]]
        #
        #   B_v2 = (I / R_wheel) * [[0], [B_21], [0], [B_41]]
        #
        # Indice : n'oublie pas de multiplier tout le vecteur colonne par
        # le facteur (I / R_wheel) avant de le renvoyer.


        A_v2 = None  # <-- à remplacer (matrice 4x4)
        B_v2 = None  # <-- à remplacer (matrice 4x1)

        # ============================================================
        # FIN TODO 5c
        # ============================================================

        # ============================================================
        # DONE 5d [BONUS] : test de contrôlabilité
        # ============================================================
        # Rappel : le système (A, B) est contrôlable si la matrice de
        # contrôlabilité Tc = [B, AB, A²B, A³B] est de rang plein (= 4
        # pour un système à 4 états).
        #
        # Construis Tc en empilant horizontalement B, A@B, A@A@B, A@A@A@B
        # (np.hstack), pour la version A_v2 / B_v2, puis vérifie son rang
        # avec np.linalg.matrix_rank(Tc).
        #
        # Affiche le résultat : si rang == 4, le système est contrôlable.
        A, B = A_v1, B_v1
        Tc = np.hstack((B, A@B, A@A@B, A@A@A@B))
        
        rang_Tc = np.linalg.matrix_rank(Tc)  # <-- à remplacer

        # ============================================================
        # FIN TODO 5d
        # ============================================================

        print(f"[BONUS] Rang de la matrice de contrôlabilité : {rang_Tc} "
              f"({'contrôlable' if rang_Tc == 4 else 'NON contrôlable'})")

        # On renvoie l'une des versions A_v1, B_v1 ou A_v2, B_v2
        return A_v1, B_v1

    def _build_system_matrices2(self):
        """
        [BONUS — optionnel] Construit A et B du système linéarisé autour de
        l'équilibre (theta=0), où la commande u est directement
        l'ACCÉLÉRATION LINÉAIRE du chariot (ddot{x}, en m/s²).
 
        Modèle identique à celui utilisé dans le firmware STM32
        (Robot_Unifie) ! C'est le modèle le plus proche de ce que tu as
        codé en C : la loi de commande y calcule directement une
        accélération demandée, pas une force ni un rapport cyclique --
        exactement comme :
 
            accel_lqr = -(K1*x + K2*x_dot + K3*theta + K4*theta_dot)
 
        Système d'état:
            dx/dt = A * x + B * u   avec u = ddot{x} (m/s²)
 
        Cette méthode n'est PAS appelée par __init__ -- c'est un exercice de
        comparaison supplémentaire, à mettre en regard de
        _build_system_matrices() (TODO 2) et _build_system_matrices1()
        (TODO 5).
 
        Returns:
            A (ndarray): Matrice d'état (4x4)
            B (ndarray): Matrice de commande (4x1)
        """
        m = self.m
        l = self.l
        g = self.g
        btheta = config.btheta
 
        # ============================================================
        # TODO 6 : Modèle "accélération commandée"
        # ============================================================
        # Intuition physique : ici, on suppose que la boucle bas-niveau
        # (moteur + asservissement de vitesse/couple) est assez rapide pour
        # que u = ddot{x} soit imposé quasi instantanément -- on ne modélise
        # donc plus l'électrique du moteur, seulement la dynamique du
        # pendule soumis à cette accélération du chariot.
        #
        # Étape a : calcule le moment d'inertie total du pendule par
        # rapport à son pivot :
        #   I_total = I + m * l^2
        # (I vient de config.I -- attention, self.m et self.l sont déjà
        # extraits en haut de la méthode, mais I n'est pas encore récupéré :
        # va le chercher dans config, comme pour btheta juste au-dessus)
        #
        # Étape b : construis la matrice A (4x4). La structure est :
        #
        #
        # Étape c : construis la matrice B (4x1) :
        #
        # Indice : np.array([[...], [...], [...], [...]]) pour chacune.
 
        I_total = None  # <-- à remplacer (étape a)
 
        A = None  # <-- à remplacer (étape b, matrice 4x4)
        B = None  # <-- à remplacer (étape c, matrice 4x1)
 
        # ============================================================
        # FIN TODO 6
        # ============================================================
 
        return A, B

    def _compute_lqr_gains(self):
        """
        Calcule les gains LQR optimaux en résolvant l'équation de Riccati algébrique.

        Résout: A'P + PA - PBR⁻¹B'P + Q = 0
        Gains: K = R⁻¹B'P

        Returns:
            K (ndarray): Matrice des gains (1x4)
        """
        try:
            # Résolution de l'équation de Riccati algébrique continue
            P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
            
            # Calcul des gains optimaux
            K = linalg.inv(self.R) @ self.B.T @ P

            return K

        except Exception as e:
            print(f"Erreur lors du calcul des gains LQR: {e}")
            return None

    def compute(self, state, target_state=None):
        """
        Calcule la commande de contrôle LQR.

        Args:
            state (array-like): Vecteur d'état actuel [x, x_dot, theta, theta_dot]
            target_state (array-like): État cible (par défaut [0, 0, 0, 0])

        Returns:
            float: Commande de contrôle u
        """
        if target_state is None:
            target_state = np.zeros(4)

        # Erreur d'état
        error = np.array(state) - np.array(target_state)

        # ============================================================
        # TODO 3 : Loi de commande LQR
        # ============================================================
        # Rappel : u = -K.(x - x_target) = -K.error
        #
        # self.K a la forme (1, 4) (une matrice ligne). 
        # Complète la ligne ci-dessous :
        
        u = -self.K@error # <-- à remplacer, doit être un float (pas un tableau)
        
        # ============================================================
        # FIN TODO 3
        # ============================================================

        return u[0]

    def get_gains(self):
        """
        Retourne les gains LQR.

        Returns:
            ndarray: Gains K (1x4)
        """
        return self.K.copy()

    def get_system_matrices(self):
        """
        Retourne les matrices du système linéarisé.

        Returns:
            tuple: (A, B) matrices
        """
        return self.A.copy(), self.B.copy()

    def check_stability(self):
        """
        Vérifie la stabilité du système en boucle fermée via les valeurs propres.

        Le système en boucle fermée est: dx/dt = (A - BK)x
        Pour que le système soit stable, toutes les valeurs propres de (A - BK)
        doivent avoir une partie réelle strictement négative.

        Returns:
            dict: {
                'stable': bool - True si le système est stable
                'eigenvalues': ndarray - Valeurs propres de (A - BK)
                'max_real_part': float - Plus grande partie réelle
            }
        """
        # ============================================================
        # TODO 4 : Analyse de stabilité par les valeurs propres
        # ============================================================
        # Étapes :
        #   1. Calcule la matrice en boucle fermée : A_cl = A - B @ K
        #   2. Calcule ses valeurs propres avec np.linalg.eigvals(A_cl)
        #      (attention : elles peuvent être complexes !)
        #   3. Récupère les parties réelles avec np.real(eigenvalues)
        #   4. La partie réelle maximale donne max_real_part
        #   5. Le système est stable si et seulement si max_real_part < 0
        #   6. Retourne le dictionnaire au format attendu (voir docstring)
        #
        # Indice : pense à utiliser self.A, self.B, self.K (déjà calculés
        # dans __init__), pas des variables locales non définies.
        
        A_cl = self.A - self.B @ self.K       # <-- à remplacer
        eigenvalues = np.linalg.eigvals(A_cl)  # <-- à remplacer
        real_parts = np.real(eigenvalues)   # <-- à remplacer
        max_real_part = max(real_parts)  # <-- à remplacer
        is_stable = (max_real_part<0)     # <-- à remplacer

        # ============================================================
        # FIN TODO 4
        # ============================================================

        return {
            'stable': is_stable,
            'eigenvalues': eigenvalues,
            'max_real_part': max_real_part
        }

    def display_control_law(self):
        """
        Affiche la loi de commande LQR sous forme explicite.

        La loi de commande est: u = -K*x = -[K_x, K_x_dot, K_theta, K_theta_dot]*[x, x_dot, theta, theta_dot]^T
        """
        K = self.K.flatten()

        print("\n" + "="*60)
        print("LOI DE COMMANDE LQR")
        print("="*60)
        print(f"\nu(t) = -K*x(t)")
        print(f"\nou K = [{K[0]:.4f}, {K[1]:.4f}, {K[2]:.4f}, {K[3]:.4f}]")
        print(f"\nSous forme developpee:")
        print(f"u(t) = -{K[0]:.4f}*x - {K[1]:.4f}*x_dot - {K[2]:.4f}*theta - {K[3]:.4f}*theta_dot")
        print("\nInterpretation:")
        print(f"  - Correction position x      : K_x        = {K[0]:.4f}")
        print(f"  - Amortissement vitesse x_dot: K_x_dot    = {K[1]:.4f}")
        print(f"  - Correction angle theta     : K_theta    = {K[2]:.4f} <- PRINCIPAL")
        print(f"  - Amortissement angulaire    : K_theta_dot= {K[3]:.4f}")
        print("="*60 + "\n")

    def print_system_info(self):
        """
        Affiche toutes les informations sur le système et le contrôleur.
        """
        print("\n" + "="*70)
        print("CONTROLEUR LQR - PENDULE INVERSE")
        print("="*70)

        # Paramètres physiques
        print("\n1. PARAMETRES PHYSIQUES:")
        print(f"   - Masse du pendule (m)    : {self.m} kg")
        print(f"   - Masse du chariot (M)    : {self.M} kg")
        print(f"   - Longueur du pendule (l) : {self.l} m")
        print(f"   - Gravite (g)             : {self.g} m/s^2")

        # Matrice A
        print("\n2. MATRICE D'ETAT A (4x4):")
        print("   dx/dt = Ax + Bu")
        with np.printoptions(precision=4, suppress=True):
            for i, row in enumerate(self.A):
                prefix = "   A = [" if i == 0 else "       ["
                suffix = "]" if i == len(self.A) - 1 else ""
                print(f"{prefix}{row}{suffix}")

        # Matrice B
        print("\n3. MATRICE DE COMMANDE B (4x1):")
        with np.printoptions(precision=4, suppress=True):
            for i, row in enumerate(self.B):
                prefix = "   B = [" if i == 0 else "       ["
                suffix = "]" if i == len(self.B) - 1 else ""
                print(f"{prefix}{row}{suffix}")

        # Matrices Q et R
        print("\n4. MATRICES DE PONDERATION:")
        print("   Q (etats) = diag([x, x_dot, theta, theta_dot]):")
        Q_diag = np.diag(self.Q)
        print(f"              {Q_diag}")
        print(f"   R (commande) = {self.R[0,0]}")

        # Gains K
        print("\n5. GAINS LQR CALCULES:")
        K = self.K.flatten()
        print(f"   K = [{K[0]:.6f}, {K[1]:.6f}, {K[2]:.6f}, {K[3]:.6f}]")
        print(f"\n   Loi de commande: u = -{K[0]:.4f}*x - {K[1]:.4f}*x_dot - {K[2]:.4f}*theta - {K[3]:.4f}*theta_dot")

        # Stabilité
        print("\n6. ANALYSE DE STABILITE:")
        stability = self.check_stability()
        stable_text = "OUI" if stability['stable'] else "NON"
        print(f"   Systeme stable: {stable_text}")
        print("\n   Valeurs propres de (A - BK):")
        for i, eig in enumerate(stability['eigenvalues'], 1):
            real_part = np.real(eig)
            imag_part = np.imag(eig)
            status = "OK" if real_part < 0 else "INSTABLE"
            if abs(imag_part) < 1e-10:
                print(f"     lambda{i} = {real_part:+.6f} [{status}]")
            else:
                print(f"     lambda{i} = {real_part:+.6f} {imag_part:+.6f}j [{status}]")
        print(f"\n   Partie reelle maximale: {stability['max_real_part']:.6f}")
        print("   (Doit etre < 0 pour la stabilite)")

        print("\n" + "="*70 + "\n")