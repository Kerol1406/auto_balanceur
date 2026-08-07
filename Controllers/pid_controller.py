"""
Correcteur PID générique.

La même classe sert DEUX fois dans le projet, montée en cascade :
  - boucle interne (rapide) : cible = angle,    mesure = theta -> sortie = commande PWM
  - boucle externe (lente)  : cible = position, mesure = x     -> sortie = angle cible

C'est pour cela que la classe ne connaît ni le robot, ni les unités : elle
manipule une erreur, point. Le câblage (qui pilote quoi, avec quel signe) est
fait dans Controllers/main_controller.py.

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

        # Mémoire du correcteur : un PID est un système à état, il se souvient
        # de ce qui s'est passé avant.
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
        # TODO 1 : calculer l'erreur e = target - current

        # TODO 2 : terme proportionnel P = Kp * e

        # TODO 3 : terme intégral. Cumuler l'erreur (self.integral += e * dt)
        #          puis I = Ki * self.integral

        # TODO 4 : terme dérivé. Approcher de/dt par (e - prev_error) / dt,
        #          puis D = Kd * derivee

        # TODO 5 : mémoriser l'erreur courante pour l'appel suivant

        # TODO 6 : renvoyer P + I + D
        raise NotImplementedError("PID.compute : à implémenter")

    def reset(self):
        """Efface la mémoire du correcteur (à appeler entre deux simulations)."""
        # TODO 7 : remettre integral et prev_error à zéro
        raise NotImplementedError("PID.reset : à implémenter")
