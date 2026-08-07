"""
Contrôleur LQR (Linear Quadratic Regulator) — retour d'état optimal.

PRINCIPE
--------
Contrairement au PID qui ne regarde qu'une grandeur à la fois, le LQR utilise
les QUATRE composantes de l'état en même temps :

    u = -K * (x - x_cible)      avec K = [K_x, K_dx, K_theta, K_dtheta]

Les gains K ne sont pas réglés à la main : ils sont CALCULÉS comme la solution
qui minimise le critère quadratique

    J = ∫ ( x'Q x + u'R u ) dt

Q pénalise les écarts d'état (rester droit, rester en place), R pénalise
l'effort moteur. Choisir Q et R, c'est arbitrer entre performance et énergie ;
c'est le seul réglage qui reste à la charge de l'ingénieur.

MODÈLE LINÉAIRE
---------------
Le calcul suppose un système linéaire  dx/dt = A x + B u, valable au voisinage
de l'équilibre (robot vertical). Deux façons d'obtenir A et B :

  a) à la main, en linéarisant les équations de robot.py autour de theta = 0
     (sin(theta) ≈ theta, cos(theta) ≈ 1, termes en dtheta^2 négligés) ;
  b) NUMÉRIQUEMENT, par différences finies sur robot.derivarives().

L'option (b) est recommandée ici : l'entrée du système est un rapport cyclique,
et la force contre-électromotrice introduit des termes d'amortissement croisés
entre dx et dtheta qui rendent le calcul à la main pénible et fragile. Surtout,
elle garantit que le modèle du contrôleur reste cohérent avec le simulateur :
si robot.py change, A et B suivent automatiquement.

À COMPLÉTER : voir les TODO.
"""

import numpy as np
from scipy import linalg

import config
from robot import Robot


class LQR:
    def __init__(self, Q=None, R=None, verbose=True):
        """
        Args:
            Q (array 4x4): pondération des états [x, dx, theta, dtheta].
                Si None, une valeur par défaut est utilisée.
            R (float ou 1x1): pondération de la commande.
            verbose (bool): affiche le récapitulatif complet à la création.
        """
        self.m = config.m
        self.M = config.M
        self.l = config.l
        self.g = config.g
        self.verbose = verbose

        if Q is None:
            # Point de départ raisonnable : on privilégie l'angle et son amortissement
            Q = np.diag([1.0, 10.0, 200.0, 2.0])
        else:
            Q = np.array(Q)

        if R is None:
            R = np.array([[1.0]])
        else:
            R = np.array([[R]]) if np.isscalar(R) else np.array(R)

        self.Q = Q
        self.R = R

        # Construction du modèle linéaire puis calcul des gains
        self.A, self.B = self._build_system_matrices()
        self.K = self._compute_lqr_gains()

        if self.verbose:
            self.print_system_info()

    def _build_system_matrices(self, eps=1e-6):
        """
        Construit A (4x4) et B (4x1) du système linéarisé autour de l'équilibre.

        Méthode conseillée : différences finies centrées sur robot.derivarives().
        Pour chaque composante j de l'état, la colonne j de A vaut

            A[:, j] = ( f(etat_eq + eps*e_j, duty_eq) - f(etat_eq - eps*e_j, duty_eq) ) / (2*eps)

        et de même pour B en perturbant le rapport cyclique.

        Returns:
            (A, B): matrices numpy de tailles (4, 4) et (4, 1)
        """
        # TODO 1 : instancier un Robot, définir l'état d'équilibre (vecteur nul)
        #          et le rapport cyclique d'équilibre (0.0)

        # TODO 2 : remplir A colonne par colonne par différences finies centrées

        # TODO 3 : calculer B en perturbant duty, et le remettre en forme (4, 1)

        # TODO 4 : renvoyer A, B
        raise NotImplementedError("LQR._build_system_matrices : à implémenter")

    def _compute_lqr_gains(self):
        """
        Calcule les gains optimaux en résolvant l'équation de Riccati algébrique.

            A'P + P A - P B R^-1 B' P + Q = 0     puis     K = R^-1 B' P

        Ne pas coder Riccati soi-même : scipy le fait déjà, et bien
        (scipy.linalg.solve_continuous_are).

        Returns:
            np.ndarray: gains K de taille (1, 4)
        """
        # TODO 5 : résoudre l'équation de Riccati avec linalg.solve_continuous_are

        # TODO 6 : en déduire K = inv(R) @ B.T @ P

        # TODO 7 : gérer proprement l'échec du solveur (certains couples Q/R
        #          n'admettent pas de solution ; l'optimiseur en essaiera).
        raise NotImplementedError("LQR._compute_lqr_gains : à implémenter")

    def compute(self, state, target_state=None):
        """
        Loi de commande : u = -K * (state - target_state).

        Args:
            state (array): état courant [x, dx, theta, dtheta]
            target_state (array): état visé (équilibre par défaut)

        Returns:
            float: rapport cyclique demandé (la saturation est faite par l'appelant)
        """
        # TODO 8 : calculer l'erreur d'état puis appliquer la loi de commande.
        #          Attention : np.dot(self.K, error) renvoie un tableau de
        #          taille 1, il faut en extraire le scalaire.
        raise NotImplementedError("LQR.compute : à implémenter")

    def check_stability(self):
        """
        Vérifie la stabilité du système bouclé.

        En boucle fermée, la dynamique devient dx/dt = (A - B K) x. Le système
        est stable si TOUTES les valeurs propres de (A - B K) ont une partie
        réelle strictement négative.

        Returns:
            dict: {'stable': bool, 'eigenvalues': ndarray, 'max_real_part': float}
        """
        # TODO 9 : construire A - B@K, calculer ses valeurs propres
        #          (np.linalg.eigvals) et renvoyer le dictionnaire attendu.
        #          Ce format est utilisé tel quel par main_controller et par
        #          Optimizers/lqr_optimizer.py : le respecter.
        raise NotImplementedError("LQR.check_stability : à implémenter")

    def get_gains(self):
        """Retourne une copie des gains K."""
        return self.K.copy()

    def get_system_matrices(self):
        """Retourne des copies de (A, B)."""
        return self.A.copy(), self.B.copy()

    # ------------------------------------------------------------------
    # Affichage (fourni : sert à vérifier votre travail)
    # ------------------------------------------------------------------

    def display_control_law(self):
        K = self.K.flatten()
        print("\n" + "=" * 60)
        print("LOI DE COMMANDE LQR")
        print("=" * 60)
        print("\nu(t) = -K*x(t)")
        print(f"\nou K = [{K[0]:.4f}, {K[1]:.4f}, {K[2]:.4f}, {K[3]:.4f}]")
        print("\nSous forme developpee:")
        print(f"u(t) = -{K[0]:.4f}*x - {K[1]:.4f}*x_dot - {K[2]:.4f}*theta - {K[3]:.4f}*theta_dot")
        print("\nInterpretation:")
        print(f"  - Correction position x      : K_x        = {K[0]:.4f}")
        print(f"  - Amortissement vitesse x_dot: K_x_dot    = {K[1]:.4f}")
        print(f"  - Correction angle theta     : K_theta    = {K[2]:.4f} <- PRINCIPAL")
        print(f"  - Amortissement angulaire    : K_theta_dot= {K[3]:.4f}")
        print("=" * 60 + "\n")

    def print_system_info(self):
        print("\n" + "=" * 70)
        print("CONTROLEUR LQR - PENDULE INVERSE")
        print("=" * 70)

        print("\n1. PARAMETRES PHYSIQUES:")
        print(f"   - Masse du pendule (m)    : {self.m} kg")
        print(f"   - Masse du chariot (M)    : {self.M} kg")
        print(f"   - Longueur du pendule (l) : {self.l} m")
        print(f"   - Gravite (g)             : {self.g} m/s^2")

        print("\n2. MATRICE D'ETAT A (4x4):")
        print("   dx/dt = Ax + Bu")
        with np.printoptions(precision=4, suppress=True):
            for i, row in enumerate(self.A):
                prefix = "   A = [" if i == 0 else "       ["
                suffix = "]" if i == len(self.A) - 1 else ""
                print(f"{prefix}{row}{suffix}")

        print("\n3. MATRICE DE COMMANDE B (4x1):")
        with np.printoptions(precision=4, suppress=True):
            for i, row in enumerate(self.B):
                prefix = "   B = [" if i == 0 else "       ["
                suffix = "]" if i == len(self.B) - 1 else ""
                print(f"{prefix}{row}{suffix}")

        print("\n4. MATRICES DE PONDERATION:")
        print("   Q (etats) = diag([x, x_dot, theta, theta_dot]):")
        print(f"              {np.diag(self.Q)}")
        print(f"   R (commande) = {self.R[0, 0]}")

        print("\n5. GAINS LQR CALCULES:")
        K = self.K.flatten()
        print(f"   K = [{K[0]:.6f}, {K[1]:.6f}, {K[2]:.6f}, {K[3]:.6f}]")

        print("\n6. ANALYSE DE STABILITE:")
        stability = self.check_stability()
        print(f"   Systeme stable: {'OUI' if stability['stable'] else 'NON'}")
        print("\n   Valeurs propres de (A - BK):")
        for i, eig in enumerate(stability['eigenvalues'], 1):
            real_part, imag_part = np.real(eig), np.imag(eig)
            status = "OK" if real_part < 0 else "INSTABLE"
            if abs(imag_part) < 1e-10:
                print(f"     lambda{i} = {real_part:+.6f} [{status}]")
            else:
                print(f"     lambda{i} = {real_part:+.6f} {imag_part:+.6f}j [{status}]")
        print(f"\n   Partie reelle maximale: {stability['max_real_part']:.6f}")
        print("   (Doit etre < 0 pour la stabilite)")
        print("\n" + "=" * 70 + "\n")
