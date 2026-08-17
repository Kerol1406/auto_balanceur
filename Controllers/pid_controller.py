"""
Correcteur PID générique.

La même classe sert DEUX fois dans le projet, montée en cascade :
  - boucle interne (rapide) : cible = angle,    mesure = theta -> sortie = commande PWM
  - boucle externe (lente)  : cible = position, mesure = x     -> sortie = angle cible

RAPPEL
------
    u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt     avec e = cible - mesure

    P : réagit à l'erreur présente  -> raideur de la réponse
    I : réagit à l'erreur passée    -> supprime l'erreur statique, mais déstabilise
    D : réagit à la tendance        -> amortit, mais amplifie le bruit des capteurs

En temps discret (pas dt fixe), l'intégrale devient une somme et la dérivée une
différence entre deux échantillons successifs.

À COMPLÉTER : voir les TODO.
"""


class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        """
        Args:
            Kp, Ki, Kd (float): gains proportionnel, intégral, dérivé
            dt (float): période d'échantillonnage (s)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current):
        """
        Calcule la commande à partir de la cible et de la mesure.

        Args:
            target (float): valeur souhaitée
            current (float): valeur mesurée

        Returns:
            float: somme des trois termes P + I + D
        """
        # DONE : calculer l'erreur, mettre à jour l'intégrale et la dérivée,
        #       puis renvoyer la commande u = P + I + D.

        erreur = target - current
        self.integral += erreur
        D = (erreur - self.prev_error)/self.dt
        self.prev_error = erreur
        return self.Kp*erreur +self.Kd*D +self.integral*self.Ki*self.dt

    def reset(self):
        """Efface la mémoire du correcteur (à appeler entre deux simulations)."""
        # DONE : remettre integral et prev_error à zéro
        self.integral = 0
        self.prev_error = 0
