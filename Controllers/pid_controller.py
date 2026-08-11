import numpy as np
import config


class PID:
    """
    PID discret avec deux raffinements indispensables ici :

    1. DERIVEE SUR LA MESURE (au lieu de la dérivée de l'erreur).
       La dérivée de l'erreur vaut d(cible - mesure)/dt. Dans une cascade, la
       cible change à CHAQUE pas (c'est la sortie de la boucle externe) : le
       terme D reçoit donc un échelon divisé par dt = 5 ms, soit un « coup de
       fouet dérivatif » permanent qui n'a rien à voir avec la dynamique du
       robot. En passant la vitesse mesurée (dx ou dtheta, disponibles dans
       l'état — et données par le gyro sur le robot réel), D ne réagit plus
       qu'au mouvement réel.

    2. ANTI-WINDUP par intégration conditionnelle.
       La sortie est saturée (rapport cyclique dans [-1, 1] pour l'angle, angle
       cible borné pour la position). Sans précaution, l'intégrale continue de
       se charger pendant la saturation et met un temps fou à se décharger
       ensuite. On gèle donc l'intégration quand la commande sature ET que
       l'erreur pousse encore dans le sens de la saturation ; on borne en plus
       l'intégrale explicitement.
    """

    def __init__(self, Kp, Ki, Kd, dt, output_max=None, integral_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_max = output_max
        self.integral_max = integral_max

        # Mémoire pour l'intégrale et la dérivée
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        """Remet la mémoire à zéro (à appeler entre deux simulations)."""
        self.integral = 0.0
        self.prev_error = None

    def compute(self, target, current, derivative=None):
        """
        Calcule la commande u en fonction de la cible et de la valeur actuelle.

        Args:
            target     : consigne
            current    : mesure
            derivative : dérivée MESURÉE de `current` (dx ou dtheta). Si elle
                         est fournie, D = -Kd * derivative (dérivée sur la
                         mesure). Sinon on retombe sur la dérivée de l'erreur,
                         par différences finies.
        """
        error = target - current

        # Terme Proportionnel
        P = self.Kp * error

        # Terme Dérivé
        if derivative is not None:
            # d(erreur)/dt = -d(mesure)/dt à consigne constante : on ignore
            # volontairement la variation de la consigne (pas de coup de fouet).
            D = -self.Kd * derivative
        else:
            prev = self.prev_error if self.prev_error is not None else error
            D = self.Kd * (error - prev) / self.dt
        self.prev_error = error

        # Terme Intégral (candidat, validé plus bas seulement s'il ne windupe pas)
        integral_essai = self.integral + error * self.dt
        if self.integral_max is not None:
            integral_essai = float(np.clip(integral_essai, -self.integral_max,
                                           self.integral_max))

        u_essai = P + self.Ki * integral_essai + D

        # Intégration conditionnelle : on n'accepte l'accumulation que si la
        # commande n'est pas saturée, ou si l'erreur tend à la désaturer.
        sature = (self.output_max is not None
                  and abs(u_essai) > self.output_max
                  and u_essai * error > 0)
        if not sature:
            self.integral = integral_essai

        u = P + self.Ki * self.integral + D
        if self.output_max is not None:
            u = float(np.clip(u, -self.output_max, self.output_max))
        return u


class PIDCascade:
    """
    Cascade PID complète : boucle externe (position) + boucle interne (angle).

    Cette classe existe pour qu'il n'y ait qu'UNE SEULE définition de la loi de
    commande. Auparavant elle était recopiée dans main_controller.py et dans
    pid_optimizer.py, et les deux copies avaient divergé : l'optimiseur réglait
    les gains AVEC la boucle de position, la simulation tournait SANS. Les gains
    étaient donc optimisés pour un système et évalués sur un autre, et le robot
    dérivait jusqu'à la chute.

    Rappel du câblage :
      - la boucle externe répond à « pour aller à x_cible, quel angle prendre ? »
        et sa sortie est bornée à PID_TARGET_THETA_MAX ;
      - la boucle interne suit cet angle ; le signe '-' vient de ce que la
        commande doit s'opposer à la chute (avancer sous le pendule).
    """

    def __init__(self, kp, ki, kd, pos_kp, pos_ki, pos_kd, dt=None,
                 u_max=None, theta_max=None):
        dt = config.dt if dt is None else dt
        u_max = config.PWM_MAX if u_max is None else u_max
        theta_max = config.PID_TARGET_THETA_MAX if theta_max is None else theta_max

        self.angle = PID(kp, ki, kd, dt,
                         output_max=u_max, integral_max=config.PID_I_MAX)
        self.pos = PID(pos_kp, pos_ki, pos_kd, dt,
                       output_max=theta_max, integral_max=config.PID_Pos_I_MAX)

    def reset(self):
        self.angle.reset()
        self.pos.reset()

    def compute(self, state, target_x=0.0):
        """
        state : [x, dx, theta, dtheta]
        Retourne (u, target_theta) : u est le rapport cyclique PWM saturé.
        """
        x, dx, theta, dtheta = state

        # 1. Boucle externe : quel angle viser pour rejoindre target_x ?
        #    Dérivée sur la mesure -> on passe dx.
        target_theta = self.pos.compute(target=target_x, current=x, derivative=dx)

        # 2. Boucle interne : suivre cet angle. Dérivée sur la mesure -> dtheta.
        u = -self.angle.compute(target=target_theta, current=theta,
                                derivative=dtheta)
        u = float(np.clip(u, -config.PWM_MAX, config.PWM_MAX))
        return u, target_theta

