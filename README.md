# Robot auto-balanceur — squelette du projet

Simulation et commande d'un robot auto-balanceur (pendule inversé sur deux
roues, plateforme YahBoom). Ce dépôt est le **squelette** du projet : la
structure, les interfaces et les explications sont fournies, la logique est à
écrire. Chaque emplacement à compléter est marqué par un `# TODO` numéroté.

## Structure

```
main.py                        Point d'entrée (fourni)          -> python main.py
config.py                      Tous les paramètres du projet
motor.py                       Modèle des moteurs (PWM -> couple)
robot.py                       Physique du robot (simulateur)
visualizer.py                  Animation du robot (fourni)

Controllers/
    main_controller.py         Orchestration de la simulation
    dummy_controller.py        Contrôleur réflexe d'exemple (fourni)
    pid_controller.py          Correcteur PID
    lqr_controller.py          Retour d'état optimal (LQR)
    fuzzy_controller.py        Logique floue (Mamdani)
    sac_controller.py          Politique apprise (Soft Actor-Critic)

Optimizers/
    pid_optimizer.py           Réglage des gains PID (twiddle)
    lqr_optimizer.py           Réglage de Q et R (differential evolution)
    fuzzy_optimizer.py         Réglage de la cascade floue
    sac_trainer.py             Entraînement de la politique SAC

Figures/                       Courbes et figures produites par le code
Ressources/                    Documents, lookup tables, politique entraînée
```

## Installation

```bash
pip install numpy scipy matplotlib
```

PyTorch (`pip install torch`) n'est nécessaire que pour la partie SAC.

## Ordre de travail conseillé

1. **`config.py`** — mesurer le robot et renseigner les paramètres physiques.
2. **`motor.py`** — vérifier avec `python motor.py` : le rendement du réducteur
   doit tomber entre 50 % et 80 %, sinon une constante est fausse.
3. **`robot.py`** — le simulateur. Test : sans commande, le robot doit tomber ;
   avec `DummyController`, il doit rester debout tout en dérivant.
4. **`Controllers/main_controller.py`** — le câblage, avec `TYPE_CONTROLEUR = "DUMMY"`.
   À partir d'ici, `python main.py` doit produire courbes et animation.
5. **`pid_controller.py`** puis **`pid_optimizer.py`** — la cascade et son réglage.
6. **`lqr_controller.py`** puis **`lqr_optimizer.py`**.
7. **`fuzzy_controller.py`** puis **`fuzzy_optimizer.py`**.
8. **`sac_controller.py`** et **`sac_trainer.py`**.

Les étapes 5 à 8 sont indépendantes entre elles : les 1 à 4 ne le sont pas.

## Conventions à respecter

- État : `state = [x, dx, theta, dtheta]`, `theta` en radians, positif quand le
  robot penche à droite.
- Commande : **rapport cyclique PWM** dans `[-1, 1]`, jamais un couple. C'est
  ce que l'on écrira réellement dans le microcontrôleur.
- Tout contrôleur expose `compute(...)` et renvoie une commande saturée.
- Aucune valeur physique en dur dans le code : tout passe par `config.py`.
