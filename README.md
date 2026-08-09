# Robot auto-balanceur : squelette du projet

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
    sac_env.py                 Environnement Gymnasium autour de robot.py
    sac_trainer.py             Entraînement de la politique SAC

Figures/                       Courbes et figures produites par le code
Ressources/                    Documents, lookup tables, politique entraînée
```

## Installation

Python **3.10 à 3.12**. Vérifiez la version avant tout le reste, c'est elle qui
servira à créer l'environnement :

```bash
python --version
```

### 1. Créer l'environnement virtuel

Un environnement virtuel isole les paquets du projet de ceux déjà installés sur
la machine : pas de conflit de versions, et tout se désinstalle en supprimant un
seul dossier. À faire une fois, à la racine du projet :

```bash
python -m venv .venv
```

### 2. L'activer

Sous Windows (PowerShell) :

```bash
.venv\Scripts\Activate.ps1
```

Si PowerShell répond que « l'exécution de scripts est désactivée », passez par
l'invite de commandes classique (`cmd`), qui n'a pas cette restriction :

```bash
.venv\Scripts\activate.bat
```

Sous macOS ou Linux :

```bash
source .venv/bin/activate
```

L'invite du terminal doit maintenant commencer par `(.venv)`. **C'est à refaire
à chaque nouveau terminal**, sinon `pip install` et `python main.py` iront taper
dans le Python du système. Pour en sortir : `deactivate`. Le dossier `.venv/`
est déjà ignoré par git, il ne partira jamais dans un commit.

### 3. Installer les dépendances

Le cœur du projet — physique, visualisation, PID, LQR, logique floue et leurs
optimiseurs — ne demande que trois paquets :

```bash
pip install numpy scipy matplotlib
```

La partie **SAC** (apprentissage par renforcement) en ajoute deux :

```bash
pip install gymnasium stable-baselines3
```

Ces deux paquets ne servent qu'à `Controllers/sac_controller.py`,
`Optimizers/sac_env.py` et `Optimizers/sac_trainer.py` : tout le reste du
projet tourne sans eux. Pour suivre les courbes d'entraînement dans
TensorBoard (facultatif) :

```bash
pip install tensorboard
```

## Ordre de travail conseillé

1. **`config.py`** : mesurer le robot et renseigner les paramètres physiques.
2. **`motor.py`** : vérifier avec `python motor.py` : le rendement du réducteur
   doit tomber entre 50 % et 80 %, sinon une constante est fausse.
3. **`robot.py`** : le simulateur. Sans commande, le robot doit tomber comme avec `DummyController`.
4. **`Controllers/main_controller.py`** : le câblage, avec `TYPE_CONTROLEUR = "DUMMY"`.
   À partir d'ici, `python main.py` doit produire courbes et animation.
5. **`pid_controller.py`** puis **`pid_optimizer.py`** : la cascade et son réglage.
6. **`lqr_controller.py`** puis **`lqr_optimizer.py`**.
7. **`fuzzy_controller.py`** puis **`fuzzy_optimizer.py`**.
8. **`sac_controller.py`** et **`sac_trainer.py`**.

Les étapes 5 à 8 sont indépendantes entre elles : les 1 à 4 ne le sont pas.

## Conventions à respecter

- État : `state = [x, dx, theta, dtheta]`, `theta` en radians, positif quand le
  robot penche à droite.
- Commande : **rapport cyclique PWM** dans `[-1, 1]`. C'est
  ce que l'on écrira réellement dans le microcontrôleur.
- Tout contrôleur expose `compute(...)` et renvoie une commande saturée.
- Aucune valeur physique en dur dans le code : tout passe par `config.py`.
