# Contrôle par logique floue d'un robot auto-balanceur

Documentation du module de contrôle flou du projet : théorie générale, application
au pendule inversé, choix d'implémentation, et fonctionnement de l'optimiseur.

**Cible matérielle** : YahBoom Self-Balancing Robot. La simulation sert à trouver
les meilleurs paramètres avant l'implémentation sur le robot réel.

---

## Table des matières

1. [Pourquoi la logique floue](#1-pourquoi-la-logique-floue)
2. [Fonctionnement général de la logique floue](#2-fonctionnement-général-de-la-logique-floue)
3. [Application au robot auto-balanceur](#3-application-au-robot-auto-balanceur)
4. [Choix d'implémentation](#4-choix-dimplémentation)
5. [L'architecture en cascade](#5-larchitecture-en-cascade-flou-dans-flou)
6. [L'optimiseur automatique](#6-loptimiseur-automatique)
7. [Résultats](#7-résultats)
8. [Utilisation](#8-utilisation)
9. [Vers l'implémentation embarquée](#9-vers-limplémentation-embarquée)
10. [Implémentation STM32 en C](#10-implémentation-stm32-en-c)
11. [Pièges rencontrés](#11-pièges-rencontrés-et-comment-les-éviter)

---

## 1. Pourquoi la logique floue

Le PID et le LQR sont des contrôleurs **analytiques** : ils exigent soit un réglage
de gains, soit un modèle mathématique du système (matrices A, B). La logique floue
suit une philosophie opposée : elle encode l'**expertise humaine** sous forme de
règles en langage naturel.

> « Si le robot penche à droite et qu'il tombe vite vers la droite,
>   alors pousse fort vers la droite. »

C'est exactement ainsi qu'un humain décrit comment tenir un manche à balai en
équilibre sur sa paume. Aucune équation différentielle n'intervient.

**Intérêt pour l'étude comparative** : trois philosophies de contrôle
fondamentalement différentes sur le même système.

| Méthode | Fondement | Ce qu'il faut fournir |
|---|---|---|
| PID | Erreur, intégrale, dérivée | 3 gains par boucle |
| LQR | Optimisation quadratique sur modèle linéarisé | Modèle + matrices Q, R |
| **Flou** | **Règles linguistiques expertes** | **Table de règles + partitions** |

---

## 2. Fonctionnement général de la logique floue

Un contrôleur flou (de type **Mamdani**, celui utilisé ici) fonctionne en trois
étapes successives.

```
   Entrées nettes                                        Sortie nette
   (angle, vitesse)                                       (commande u)
        │                                                      ▲
        ▼                                                      │
  ┌───────────┐      ┌────────────┐      ┌──────────────┐      │
  │FUZZIFIER  │─────▶│ INFÉRENCE  │─────▶│ DÉFUZZIFIER  │──────┘
  └───────────┘      └────────────┘      └──────────────┘
   degrés             activation des      centre de
   d'appartenance     classes de sortie   gravité
```

### 2.1 Fuzzification — du chiffre au mot

En logique booléenne, un angle de 9° est soit « petit » soit « grand ». En logique
floue, il peut être « petit à 30 % **et** grand à 70 % ». Chaque grandeur physique
est décrite par des **fonctions d'appartenance** (ici des trapèzes et triangles)
qui donnent, pour une valeur numérique, le degré d'appartenance µ ∈ [0, 1] à
chaque classe linguistique.

```
 µ
 1 ┤ ╲EG      ╱╲G      ╱╲C╱╲      D╱╲      ╱ED
   │  ╲      ╱  ╲     ╱  ╳  ╲    ╱  ╲     ╱
 0 ┼───╲────╱────╲───╱───────╲──╱────╲───╱────▶ angle
   -30 -20  -10        0        10   20   30
```

Un angle de -15° donne par exemple : EG = 0.5, G = 0.5, C = 0, D = 0, ED = 0.

### 2.2 Inférence — appliquer les règles

Chaque combinaison (classe d'angle × classe de vitesse) constitue une règle qui
désigne une classe de sortie. Avec 5 classes par entrée, cela fait **25 règles**
rangées dans une table.

Pour chaque règle on calcule sa **force d'activation** — le « ET » flou est
implémenté par le **minimum** :

```
force(règle i,j) = min( µ_angle[j] , µ_vitesse[i] )
```

Plusieurs règles pouvant désigner la même classe de sortie, on les **agrège** par
le **maximum** :

```
activation[classe] = max( force de toutes les règles menant à cette classe )
```

### 2.3 Défuzzification — revenir au chiffre

L'inférence produit un degré d'activation par classe de sortie ; il faut en tirer
une valeur numérique unique. Deux variantes existent :

**(a) Mamdani complet** — chaque classe de sortie est un triangle qu'on découpe à
son niveau d'activation ; on agrège les découpes (max) et on calcule le centroïde
de la surface obtenue.

**(b) Méthode des singletons (retenue ici)** — chaque classe est réduite à son
centre de gravité `c_k` ; la sortie est la moyenne des centres pondérée par les
activations :

```
        Σ activation[k] × c[k]
   u = ────────────────────────
          Σ activation[k]
```

La figure `Figures/fuzzy_3_defuzzification.png` superpose les deux : les polygones
découpés avec leur centroïde (orange) et le résultat par singletons (vert). Les
deux valeurs sont quasi confondues, ce qui justifie la variante (b), bien plus
légère à calculer.

---

## 3. Application au robot auto-balanceur

### 3.1 Variables linguistiques

**Entrée 1 — angle d'inclinaison θ (degrés)**, 5 classes :

| Classe | Signification | Support |
|---|---|---|
| EG | Extrême Gauche | θ < -10° |
| G | Gauche | -20° < θ < 0° |
| C | Centre (vertical) | -3° < θ < 3° |
| D | Droite | 0° < θ < 20° |
| ED | Extrême Droite | θ > 10° |

**Entrée 2 — vitesse angulaire (rad/s)**, 5 classes :

| Classe | Signification | Support |
|---|---|---|
| TED | Tombe Extrême Droite | v < -1.5 |
| TD | Tombe Droite | -3 < v < 0 |
| E | Équilibre | -0.5 < v < 0.5 |
| TG | Tombe Gauche | 0 < v < 3 |
| TEG | Tombe Extrême Gauche | v > 1.5 |

> **Attention aux conventions de signe** : dans le fuzzifier, θ > 0 signifie
> « penché à Droite », mais une vitesse angulaire **négative** signifie « Tombe
> Droite ». Les deux entrées ont donc des conventions opposées. C'est pourquoi le
> code envoie `-θ̇` au contrôleur (voir §4.3).

**Sortie — commande moteur**, 5 classes : `GV` (Gauche Vite), `GD` (Gauche
Doucement), `R` (Repos), `DD` (Droite Doucement), `DV` (Droite Vite).

### 3.2 La table de règles

Transcrite depuis le carnet manuscrit de conception. Lignes = vitesse angulaire,
colonnes = angle, cases = classe de sortie :

|          | **EG** | **G** | **C** | **D** | **ED** |
|----------|--------|-------|-------|-------|--------|
| **TED**  | DV     | DD    | GD    | GV    | GV     |
| **TD**   | DV     | DD    | R     | GD    | GV     |
| **E**    | DV     | DD    | R     | GD    | GV     |
| **TG**   | DV     | DD    | R     | GD    | GV     |
| **TEG**  | DV     | DV    | DD    | GD    | GV     |

**Lecture d'une case** : ligne TD, colonne D → « SI le robot penche à Droite ET
qu'il tombe vers la Droite, ALORS pousse Gauche Doucement » (les roues avancent
sous le centre de gravité pour le rattraper).

La ligne **E** mérite un commentaire. Elle contenait initialement « Repos » partout
— logique en apparence : si le robot ne bouge pas, ne rien faire. Mais cela crée
une **zone morte** : un robot penché à 15° mais momentanément immobile ne recevait
aucune correction. Elle a été remplacée par une réaction sur l'angle seul, ce qui
a réduit l'oscillation résiduelle.

---

## 4. Choix d'implémentation

Fichier principal : [`Controllers/fuzzy_controller.py`](../Controllers/fuzzy_controller.py)

### 4.1 Structure de la classe

```python
class FuzzyController:
    OUTPUT_LABELS = ['GV', 'GD', 'R', 'DD', 'DV']
    RULE_TABLE    = np.array([...])   # 5×5, indices dans OUTPUT_LABELS

    def __init__(self, state, output_centers, input_gains, verbose=False)
    def compute(self, state)          # interface pull-and-plug (comme LQR/PID)
    def fuzzify(self, input_value)    # -> matrice 2×5 de degrés d'appartenance
    def inference(self, fuzzy_value)  # -> vecteur 5 d'activations
    def defuzzify(self, inferred)     # -> commande scalaire
    def extract_lookup_table(...)     # pré-calcul pour l'embarqué
    def export_pipeline_figures(...)  # visualisation des 4 étapes
```

### 4.2 Les gains d'entrée : rendre le contrôleur réglable

Les fonctions d'appartenance sont **figées** (c'est l'expertise du carnet). Pour
que l'optimiseur puisse quand même adapter le contrôleur sans les redessiner, les
entrées sont multipliées par des **gains** avant fuzzification :

```python
scaled_state = [state[0] * input_gains[0], state[1] * input_gains[1]]
```

Un gain de 2 sur l'angle revient à **contracter** l'univers de discours (le
contrôleur devient deux fois plus réactif) sans toucher à une seule ligne des
fonctions d'appartenance. C'est le levier de réglage standard en commande floue.

De même, les **centres de sortie** (`output_centers`) sont paramétrables : ils
fixent l'amplitude de la commande associée à chaque classe.

**Bilan : 4 paramètres réglables par boucle**, la structure de connaissance
(règles + partitions) restant intacte.

### 4.3 Câblage dans la boucle de simulation

Trois conversions sont nécessaires entre l'état de la simulation
`[x, ẋ, θ, θ̇]` (SI, radians) et le contrôleur flou :

```python
u = -fuzzy_controller.compute([np.degrees(theta - target_theta), -theta_dot])
```

| Élément | Raison |
|---|---|
| `np.degrees(...)` | le fuzzifier raisonne en degrés, l'état est en radians |
| `-theta_dot` | conventions de signe opposées entre les deux entrées (§3.1) |
| `-` devant le résultat | la physique demande u > 0 pour θ > 0 (identique au PID) |

### 4.4 Visualisation du pipeline

`export_pipeline_figures()` exporte quatre PNG dans `Figures/` :

| Fichier | Contenu |
|---|---|
| `fuzzy_1_fuzzification.png` | fonctions d'appartenance + point d'entrée courant |
| `fuzzy_2_inference.png` | matrice des forces des 25 règles + activations de sortie |
| `fuzzy_3_defuzzification.png` | polygones découpés, surface agrégée, centroïdes |
| `fuzzy_4_surface_commande.png` | surface de commande complète u = f(θ, θ̇) |

> **Note de conception** : la version initiale affichait les courbes à chaque appel
> de `fuzzify()`, c'est-à-dire 2000 fenêtres matplotlib par simulation. Les tracés
> sont désormais séparés du calcul et exportés à la demande.

### 4.5 Lookup table

`extract_lookup_table()` balaye une grille (entrée 1 × entrée 2) et pré-calcule la
commande en chaque point ; `lookup_control()` fait ensuite une interpolation
bilinéaire, et `load_lookup_table()` relit un CSV sauvegardé. Cela remplace
fuzzification + inférence + défuzzification par quelques multiplications — voir §9.

> **Piège corrigé** : cette méthode appelait initialement `fuzzify()` directement,
> court-circuitant les `input_gains`. La table se retrouvait indexée en coordonnées
> « après gain » alors qu'on l'interroge avec des mesures brutes — avec des gains
> de 0.31 et 5.48, l'écart était considérable. Elle passe désormais par
> `compute()`, qui applique les gains.

---

## 5. L'architecture en cascade (flou dans flou)

### 5.1 Le problème

Un contrôleur flou sur `[θ, θ̇]` maintient parfaitement la verticalité **mais ne
contrôle pas la position** : le robot reste droit tout en dérivant indéfiniment.
Il faut une seconde boucle qui ramène x vers 0.

### 5.2 La solution retenue

Une **cascade à deux boucles floues**, symétrique du PID-dans-PID déjà présent
dans le projet :

```
 x, ẋ ──▶┌────────────────────┐ θ_cible  ┌────────────────────┐──▶ u ──▶ Robot
         │ BOUCLE EXTERNE     │─────────▶│ BOUCLE INTERNE     │
         │ (floue) « lente »  │  (rad)   │ (floue) « rapide » │
         │ position → angle   │          │ angle → commande   │
         └────────────────────┘          └────────────────────┘
              le Stratège                      le Muscle
```

La boucle externe **réutilise la même table de règles et les mêmes fonctions
d'appartenance** que la boucle interne. Seuls changent :
- les **gains d'entrée**, qui projettent des mètres et m/s dans l'univers du fuzzifier ;
- les **centres de sortie**, qui sont ici des **angles cibles** (rad) et non des commandes moteur.

Sémantiquement, la règle « SI je suis trop à gauche ET je dérive vers la gauche,
ALORS penche fort vers la droite » a exactement la même structure que la règle
d'angle. **Zéro règle supplémentaire à écrire.**

La sortie est saturée à ±0.17 rad (~10°) : au-delà, le robot accélérerait trop
brutalement et tomberait.

### 5.3 Pourquoi pas un flou unique à 4 entrées ?

C'était l'alternative naturelle : `[x, ẋ, θ, θ̇] → u`, l'équivalent flou du retour
d'état complet du LQR.

| Critère | Cascade (retenue) | Flou 4 entrées |
|---|---|---|
| Nombre de règles | 25, réutilisées 2× | **5⁴ = 625** |
| Écriture des règles | intuitives, depuis le carnet | ingérables à la main |
| Paramètres à optimiser | 8 | plusieurs dizaines |
| Visualisation | 2 surfaces 2D | surface 4D invisualisable |
| Embarqué | 2 tables 2D | table 4D (~Mo) |
| Couplages entre variables | ignorés (sous-optimal) | exploités |
| Hypothèse requise | séparation des échelles de temps | aucune |

Une règle à 4 antécédents (« SI x est G ET ẋ est TD ET θ est D ET θ̇ est TEG… »)
n'a plus de sens intuitif : en pratique on générerait ces 625 cases par une
formule, ce qui reviendrait à ré-encoder un LQR en plus lourd — en perdant
précisément ce qui fait l'intérêt du flou, l'expertise humaine lisible.

**Le contre de la cascade** : elle suppose que la boucle interne (angle, ~0.1 s)
soit nettement plus rapide que l'externe (position, ~s). Si l'externe devient trop
agressive, les deux boucles interfèrent et le système diverge. C'est un piège
réellement rencontré (§10), résolu en optimisant les deux boucles **conjointement**.

Le pendule inversé se décompose naturellement en « angle rapide / position lente » :
c'est le cas favorable à la cascade.

---

## 6. L'optimiseur automatique

Fichier : [`Optimizers/fuzzy_optimizer.py`](../Optimizers/fuzzy_optimizer.py)

### 6.1 Ce qui est optimisé — et ce qui ne l'est pas

**Restent figés** (l'expertise humaine) : la table de règles et les fonctions
d'appartenance.

**Sont optimisés** — 8 paramètres continus :

| # | Paramètre | Rôle |
|---|---|---|
| 1-2 | `k_angle`, `k_vitesse` | gains d'entrée de la boucle interne |
| 3-4 | `c1`, `c2` | centres de sortie internes, symétriques `[-c2,-c1,0,c1,c2]` |
| 5-6 | `pk_x`, `pk_dx` | gains d'entrée de la boucle externe |
| 7-8 | `pc1`, `pc2` | centres de sortie externes (angles cibles, rad) |

La symétrie des centres est imposée par construction : elle divise par deux la
dimension du problème et garantit un comportement identique à gauche et à droite.

### 6.2 Algorithme : évolution différentielle

`scipy.optimize.differential_evolution`, le même que le `LQRAutoTuner`, pour que la
comparaison entre méthodes reste équitable.

**Pourquoi pas une descente de gradient ?** La fonction de coût n'est pas
différentiable : elle contient des saturations (`np.clip`), des minimums et
maximums (l'inférence floue), et surtout des **discontinuités de crash** (un
paramètre légèrement différent fait tomber le robot, le coût saute). L'évolution
différentielle ne demande aucun gradient et explore globalement — indispensable ici,
où le paysage de coût comporte plusieurs bassins séparés par des zones de crash.

### 6.3 La fonction de coût

Chaque jeu de paramètres est évalué par une simulation complète de 20 s (2000 pas),
qui produit :

```python
cost = ( 10.0 * settling_time_theta      # convergence rapide de l'angle
       +  5.0 * settling_time_x          # convergence rapide de la position
       + 50.0 * total_angle_error        # erreur angulaire cumulée (∫θ²)
       + 10.0 * total_pos_error          # erreur de position cumulée (∫x²)
       + 0.01 * total_effort             # économie d'énergie (∫u²)
       + 20.0 * max_overshoot_theta      # dépassement angulaire
       + 300.0 * osc_theta               # ── oscillation résiduelle d'angle
       + 100.0 * osc_x                   # ── oscillation résiduelle de position
       + 300.0 * mean_x_tail )           # ── biais statique de position
```

Les six premiers termes sont **identiques au `LQRAutoTuner`** (comparaison
équitable). Les trois derniers, spécifiques, ciblent le **cycle limite** : mesurés
sur le **dernier quart** de la simulation, ils pénalisent un robot qui se dandine
indéfiniment au lieu de s'immobiliser.

Ces trois termes ont été ajoutés après observation : sans eux, l'optimiseur
trouvait des solutions au bon score global mais qui oscillaient en permanence. Le
poids de `mean_x_tail` a dû être porté de 50 à 300 pour éliminer un biais statique
résiduel de 0.5 m.

### 6.4 Gestion des crashs

Un jeu de paramètres qui fait tomber le robot ne reçoit pas une pénalité constante,
mais une **pénalité graduée** selon le temps de survie :

```python
return 1e5 + 1e5 * (1.0 - crash_time / sim_time)
```

Sans cette gradation, toutes les solutions instables auraient le même coût et
l'algorithme n'aurait aucune direction de progrès depuis les zones instables. Ici,
« tomber au bout de 5 s » vaut mieux que « tomber au bout de 0.5 s », ce qui guide
la population vers les régions stables.

### 6.5 Sauvegarde des résultats

`update_config_file()` réécrit les lignes `FUZZY_*` de `config.py` en préservant les
commentaires. Appelée automatiquement quand `FAIRE_AUTOTUNING = True`. Les
paramètres optimaux sont donc réutilisés directement au lancement suivant.

> Le script de test `Ressources/test_fuzzy.py` **n'écrit pas** dans `config.py`
> (il sauvegarde dans `Figures/fuzzy_optimized_params.txt`) : il doit pouvoir
> comparer « avant / après » sans modifier la référence à chaque exécution.

---

## 7. Résultats

Conditions : robot lâché à 10° d'inclinaison, 1 m de la cible, simulation de 20 s,
paramètres physiques du YahBoom.

| Métrique | Avant optimisation | Après optimisation |
|---|---|---|
| Oscillation résiduelle θ | ±23° (quasi instable) | **±0.05°** |
| Oscillation résiduelle x | ±1.5 m | **0 mm** |
| Position finale | dérive continue | **0.000 m** |
| Stabilisation θ / x | jamais | **9.5 s / 8.7 s** |
| Effort moteur (RMS / max) | 3.93 | 0.67 / 1.59 |

**Paramètres optimaux obtenus** :

```python
FUZZY_OUTPUT_CENTERS     = np.array([-7.3844, -0.6842, 0., 0.6842, 7.3844])
FUZZY_INPUT_GAINS        = np.array([0.2200, 2.5658])
FUZZY_POS_OUTPUT_CENTERS = np.array([-0.1088, -0.0318, 0., 0.0318, 0.1088])
FUZZY_POS_INPUT_GAINS    = np.array([5.8376, 15.6403])
```

**Lecture physique de la solution** : les centres intérieurs sont très faibles
(±0.68) face aux centres extrêmes (±7.4). Près de l'équilibre le contrôleur agit
donc comme un gain fin quasi linéaire — ce qui supprime l'effet « paliers »
responsable du cycle limite — tout en gardant une forte réserve d'autorité pour
les vraies chutes. Côté position, un gain élevé sur x (5.84) est rendu stable par
un fort amortissement sur ẋ (15.64).

**Limite connue** : sans action intégrale, la convergence en position est plus
lente qu'avec un PID externe (8.7 s contre ~3 s). C'est le prix méthodologique
d'une cascade 100 % floue.

> **Les paramètres optimaux dépendent de l'état initial** utilisé pendant
> l'optimisation. Depuis un état plus proche de l'équilibre (`[0.1, 0, -0.2, 0]`,
> celui de `main.py`), l'optimiseur converge vers un jeu différent — centres
> internes plus doux (±0.51/±3.23) et gain de position plus fort (12.5) — qui
> stabilise en 2.7 s au lieu de 8.7 s. Il est donc recommandé d'optimiser depuis
> l'état initial le plus représentatif des conditions réelles d'utilisation,
> ou depuis plusieurs états si l'on cherche un réglage robuste.

---

## 8. Utilisation

### Lancer une simulation

Dans [`main.py`](../main.py) :

```python
TYPE_CONTROLEUR = "FUZZY"     # "PID", "LQR" ou "FUZZY"
FAIRE_AUTOTUNING = False      # True = optimise puis écrit dans config.py
FAIRE_VISUALISATION = True    # animation du robot
state = np.array([0.1, 0.0, -0.2, 0.0])   # [x, ẋ, θ, θ̇]
```

```bash
python main.py
```

### Générer le rapport complet

```bash
python Ressources/test_fuzzy.py
```

Enchaîne : simulation de référence → export des 4 figures du pipeline →
auto-tuning → simulation optimisée → figures optimisées. Tout est écrit dans
`Figures/` (PNG, CSV des trajectoires, et paramètres en `.txt`).

### Optimiser seul

```bash
python -m Optimizers.fuzzy_optimizer
```

### Fichiers du module

| Fichier | Rôle |
|---|---|
| `Controllers/fuzzy_controller.py` | le contrôleur (fuzzification, inférence, défuzzification, lookup, figures) |
| `Controllers/main_controller.py` | branchement de la cascade dans la boucle de simulation |
| `Optimizers/fuzzy_optimizer.py` | auto-tuning et écriture dans `config.py` |
| `Ressources/test_fuzzy.py` | script de test et de génération du rapport |
| `Ressources/LookupTables/` | tables pré-calculées pour l'embarqué (générées automatiquement) |
| `Ressources/export_c_tables.py` | conversion des tables CSV en tableaux C (§10.2) |
| `Ressources/STM32/` | module C de référence pour le robot réel (§10) |
| `config.py` | paramètres `FUZZY_*` |

---

## 9. Vers l'implémentation embarquée

Sur le YahBoom, le contrôleur doit tourner dans une boucle temps réel (~10 ms).
Deux stratégies :

**(a) Calcul direct** — porter fuzzification/inférence/défuzzification. Environ
25 minimums, 25 maximums et une division par cycle et par boucle : tout à fait
supportable sur un microcontrôleur moderne, et cela garde les règles modifiables
à chaud.

**(b) Lookup table (recommandé)** — pré-calculer hors ligne la surface de commande
et n'embarquer que deux interpolations bilinéaires (une par boucle).

Les tables sont **générées et sauvegardées automatiquement** à chaque lancement
d'une simulation floue, dans `Ressources/LookupTables/` :

| Fichier | Contenu |
|---|---|
| `fuzzy_lookup_angle.csv` | boucle interne : `[θ (deg), θ̇ (rad/s)] → u` |
| `fuzzy_lookup_position.csv` | boucle externe : `[x (m), ẋ (m/s)] → θ_cible (rad)` |
| `fuzzy_lookup_metadata.txt` | paramètres et table de règles ayant généré les CSV |

Le script `test_fuzzy.py` produit en plus les variantes `fuzzy_optimized_*`.

**Format CSV** : première ligne = valeurs de l'entrée 1, première colonne =
valeurs de l'entrée 2, le reste = sortie (la case [0,0] est inutilisée). C'est un
format directement lisible par un script de conversion vers un tableau C.

```python
# Génération (automatique, mais appelable à la main)
save_cascade_lookup_tables(fuzzy_interne, fuzzy_externe)

# Relecture et utilisation
a, v, table = FuzzyController.load_lookup_table('Ressources/LookupTables/fuzzy_lookup_angle.csv')
u = FuzzyController.lookup_control(theta_deg, -theta_dot, a, v, table)
```

> **Les tables sont indexées par les entrées BRUTES des capteurs.** Les
> `input_gains` sont déjà incorporés dans les valeurs stockées : côté embarqué on
> interroge donc la table directement avec les mesures, sans reproduire la mise à
> l'échelle. La plage de la table externe est calculée automatiquement à partir des
> gains (`±30/k_x` mètres, `±5/k_dx` m/s) pour couvrir exactement l'univers utile.

**Fidélité validée** : une simulation pilotée *uniquement* par les deux tables
interpolées converge au même point que le contrôleur flou complet
(x = +0.006 m contre -0.009 m ; θ = -0.06° contre +0.03°). L'erreur d'interpolation
maximale est de 1.7×10⁻² sur la commande (~0.5 % de la plage) et 8×10⁻⁴ rad sur
l'angle cible. Une grille 61×41 en `float32` occupe ~10 ko par boucle — négligeable
— et le coût par cycle tombe à quelques multiplications.

### Points d'attention pour le portage

1. **Commande** : la simulation intègre désormais un modèle des moteurs
   ([`motor.py`](../motor.py)), si bien que `u` est directement le rapport
   cyclique PWM dans [-1, 1]. Reste à vérifier les caractéristiques moteur de
   `config.py` (rapport de réduction notamment) sur la fiche du kit.
2. **Bruit des capteurs** : θ̇ vient du gyroscope et ẋ des encodeurs. La boucle
   externe utilise un gain élevé sur ẋ (15.6), ce qui amplifie le bruit — prévoir
   un filtre passe-bas ou une fusion de capteurs (filtre complémentaire / Kalman).
3. **Fidélité du modèle** : les paramètres optimaux dépendent des valeurs de
   `config.py` (masses, inerties, frottements). Les frottements `bx` et `btheta`
   sont des estimations ; il faudra probablement ré-optimiser après identification
   sur le robot réel.
4. **Offset de l'IMU** : un biais de quelques dixièmes de degré sur θ crée une
   dérive lente. Calibrer le zéro au démarrage, robot maintenu vertical.

Le portage complet en C pour le STM32 — arborescence, code, génération des tables
et procédure de calibration — fait l'objet de la [section 10](#10-implémentation-stm32-en-c).

---

## 10. Implémentation STM32 en C

Cette section décrit le portage complet sur le microcontrôleur STM32 du YahBoom.
Le code de référence est fourni dans `Ressources/STM32/` : il compile sans
avertissement (`gcc -std=c99 -Wall -Wextra`) et a été validé numériquement contre
la simulation Python.

### 10.1 Organisation des fichiers

L'arborescence C reprend celle de la simulation, pour que la correspondance entre
les deux soit immédiate :

```
Simulation (Python)                    Embarqué (C)
───────────────────                    ────────────
Controllers/fuzzy_controller.py   ──▶  Controllers/fuzzy_controller.{c,h}
  (cascade + inférence)                  (cascade seule, tables pré-calculées)
                                  ──▶  Controllers/fuzzy_lookup.{c,h}
                                         (interpolation bilinéaire)
Ressources/LookupTables/*.csv     ──▶  Controllers/fuzzy_tables.{c,h}
                                         (généré, ne pas éditer)
config.py                         ──▶  #define dans fuzzy_tables.h
main.py                           ──▶  main.c (boucle temps réel 10 ms)
```

Fichiers livrés dans `Ressources/STM32/` :

| Fichier | Rôle | Édition |
|---|---|---|
| `fuzzy_lookup.h` / `.c` | interpolation bilinéaire générique | manuelle |
| `fuzzy_controller.h` / `.c` | cascade des deux boucles floues | manuelle |
| `fuzzy_tables.h` / `.c` | tables + constantes | **généré** |

### 10.2 Génération des tables C

Le script `Ressources/export_c_tables.py` convertit les CSV en tableaux C :

```bash
python Ressources/export_c_tables.py
```

```bash
python Ressources/export_c_tables.py --prefix fuzzy_optimized --decimation 2
```

- `--prefix` choisit le jeu de tables (`fuzzy` ou `fuzzy_optimized`) ;
- `--decimation 2` sous-échantillonne la grille et divise l'empreinte par 4.

Le script vérifie que les axes sont à pas constant, puis n'émet que la valeur
minimale et le pas de chaque axe : côté embarqué l'indice se calcule directement,
**sans recherche**, ce qui donne une interpolation à coût constant.

À refaire après **chaque** ré-optimisation, sinon le robot tourne avec d'anciens
paramètres.

### 10.3 Le cœur : interpolation bilinéaire

`fuzzy_lookup.h` décrit une table à pas constant :

```c
typedef struct {
    const float *data;   /* n2 lignes x n1 colonnes, row-major */
    float        min1;   /* premiere valeur de l'entree 1 */
    float        step1;  /* pas de l'entree 1 */
    float        min2;
    float        step2;
    uint16_t     n1;
    uint16_t     n2;
} FuzzyTable;
```

L'interpolation complète, qui remplace fuzzification + inférence +
défuzzification :

```c
float fuzzy_lookup(const FuzzyTable *t, float x1, float x2)
{
    float f1, f2, a, b, v00, v01, v10, v11, low, high;
    uint16_t i1, i2;
    uint32_t base;

    /* Position continue dans la grille */
    f1 = (x1 - t->min1) / t->step1;
    f2 = (x2 - t->min2) / t->step2;

    /* Saturation aux bords du domaine (equivalent de np.clip) */
    f1 = fuzzy_clampf(f1, 0.0f, (float)(t->n1 - 1));
    f2 = fuzzy_clampf(f2, 0.0f, (float)(t->n2 - 1));

    /* Indice de la cellule (borne pour garder i+1 valide) */
    i1 = (uint16_t)f1;
    i2 = (uint16_t)f2;
    if (i1 > (uint16_t)(t->n1 - 2)) { i1 = (uint16_t)(t->n1 - 2); }
    if (i2 > (uint16_t)(t->n2 - 2)) { i2 = (uint16_t)(t->n2 - 2); }

    a = f1 - (float)i1;   /* poids fractionnaires */
    b = f2 - (float)i2;

    base = (uint32_t)i2 * t->n1 + i1;   /* les 4 sommets de la cellule */
    v00 = t->data[base];
    v01 = t->data[base + 1u];
    v10 = t->data[base + t->n1];
    v11 = t->data[base + t->n1 + 1u];

    low  = v00 + (v01 - v00) * a;       /* interpolation sur l'entree 1 */
    high = v10 + (v11 - v10) * a;
    return low + (high - low) * b;      /* puis sur l'entree 2 */
}
```

Coût : 2 divisions, ~10 multiplications, 4 accès flash. Négligeable devant les
10 ms de la boucle, même sans FPU.

### 10.4 La cascade

`fuzzy_controller.c` transpose exactement le câblage de `main_controller.py`,
conventions de signe comprises :

```c
#define RAD_TO_DEG 57.295779513082320876f

float fuzzy_cascade_target_theta(const FuzzyCascade *c, float x, float dx)
{
    float target;

    /* Boucle externe : "pour revenir a x = 0, quel angle dois-je prendre ?"
     * Le signe '-' sur dx reproduit la convention de la simulation. */
    target = fuzzy_lookup(&c->position, x, -dx);

    /* Securite : angle cible borne, sinon le robot accelere trop fort. */
    return fuzzy_clampf(target, -c->target_theta_max, c->target_theta_max);
}

float fuzzy_cascade_compute(const FuzzyCascade *c,
                            float x, float dx, float theta, float dtheta)
{
    float theta_corrige, target, erreur_deg, u;

    theta_corrige = theta - c->theta_offset;      /* offset IMU */

    /* 1. Boucle externe (lente) : position -> angle cible */
    target = fuzzy_cascade_target_theta(c, x, dx);

    /* 2. Boucle interne (rapide) : erreur d'angle -> commande moteur.
     *    La table raisonne en degres alors que l'etat est en radians.
     *    Le signe '-' final vient de la physique : u s'oppose a la chute. */
    erreur_deg = (theta_corrige - target) * RAD_TO_DEG;
    u = -fuzzy_lookup(&c->angle, erreur_deg, -dtheta);

    return fuzzy_clampf(u, -c->u_max, c->u_max);
}
```

> **Les trois conversions du §4.3 sont toutes présentes** : degrés, `-dtheta`,
> et le signe négatif final. En oublier une donne un robot qui tombe
> immédiatement ou qui accélère dans le mauvais sens.

### 10.5 Intégration dans la boucle temps réel

```c
/* main.c - boucle de controle a 100 Hz (dt = 10 ms, identique a config.dt) */
#include "fuzzy_controller.h"

static FuzzyCascade cascade;

void controle_init(void)
{
    fuzzy_cascade_init(&cascade);
    cascade.theta_offset = imu_calibrer_zero();  /* robot maintenu vertical */
}

/* Appelee par l'interruption du timer toutes les 10 ms */
void controle_tick(void)
{
    float x, dx, theta, dtheta, u;
    int16_t pwm;

    /* 1. Lecture des capteurs */
    x      = encodeurs_position_m();      /* m     */
    dx     = encodeurs_vitesse_ms();      /* m/s   */
    theta  = imu_angle_rad();             /* rad, filtre complementaire */
    dtheta = imu_vitesse_angulaire();     /* rad/s, gyroscope */

    /* 2. Securite : au-dela de 45 deg le robot est tombe, on coupe */
    if (theta > 0.785f || theta < -0.785f) {
        moteurs_stop();
        return;
    }

    /* 3. Commande floue */
    u = fuzzy_cascade_compute(&cascade, x, dx, theta, dtheta);

    /* 4. u EST deja le rapport cyclique dans [-1, 1] : il ne reste qu'a le
     *    convertir en valeur de registre du timer (voir 10.6). */
    pwm = (int16_t)(u * PWM_PERIODE);
    moteurs_appliquer(pwm, pwm);
}
```

### 10.6 Calibration : ce qu'il reste à mesurer sur le robot

La simulation donne les paramètres ; ces quatre points ne peuvent se régler que
sur le matériel.

1. **Conversion en registre timer** — depuis que la simulation modélise les
   moteurs (`motor.py`), la sortie du contrôleur **est** le rapport cyclique dans
   [-1, 1]. Il n'y a donc plus de facteur d'échelle inconnu à identifier : il
   suffit de multiplier par la période du timer PWM (`PWM_PERIODE`, la valeur du
   registre ARR) et d'appliquer le signe au sens de rotation du pont en H.
   En revanche, **vérifier le rapport de réduction des moteurs** sur la fiche du
   kit : `config.py` est réglé sur un JGB37-520 **1:30** (333 tr/min à vide,
   4.9 kg·cm). Un rapport différent change le couple et la vitesse maximale, donc
   les paramètres optimaux — il faut alors corriger `config.py` et ré-optimiser.
   Voir [`Rapport_Corrections_et_Portage_C.md`](Rapport_Corrections_et_Portage_C.md), section B0.
2. **Offset de l'IMU** (`theta_offset`) — un biais de quelques dixièmes de degré
   suffit à créer une dérive lente. Calibrer au démarrage, robot maintenu vertical
   pendant 1 à 2 s, en moyennant les mesures.
3. **Filtrage de `dx`** — la boucle externe utilise un gain élevé sur la vitesse
   linéaire (15.6). Le signal des encodeurs différencié est bruité : prévoir un
   passe-bas du premier ordre, sans quoi le bruit sera amplifié dans l'angle cible.
4. **Frottements réels** — `bx` et `btheta` de `config.py` sont des estimations.
   Après identification sur le robot, ré-optimiser puis régénérer les tables.

### 10.7 Empreinte et validation

**Mémoire** : deux grilles 41×61 en `float32`, soit 9.8 ko chacune, **19.5 ko de
flash** au total. Confortable sur un STM32F103 (64 à 256 ko), et divisible par 4
via `--decimation 2` si nécessaire. Aucune allocation dynamique, aucune RAM
au-delà des quelques flottants de la structure.

**Validation numérique** : le module C a été compilé et exécuté sur un jeu d'états
de test, puis comparé à la simulation Python. Écart maximal :

| Grandeur | Écart max C ↔ Python |
|---|---|
| Angle cible `θ_cible` | 2.0×10⁻³ rad (0.11°) |
| Commande `u` | 2.4×10⁻² (~0.7 % de la plage) |

Ces écarts correspondent **exactement** à l'erreur d'interpolation de la grille
mesurée côté Python — il n'y a donc aucune divergence d'implémentation entre les
deux versions. Une simulation entièrement pilotée par les tables converge au même
point que le contrôleur flou complet (§9).

**Reproduire la validation** :

```bash
gcc -std=c99 -Wall -Wextra -O2 -I Ressources/STM32 votre_test.c \
    Ressources/STM32/fuzzy_lookup.c Ressources/STM32/fuzzy_controller.c \
    Ressources/STM32/fuzzy_tables.c -o test -lm
```

### 10.8 Note sur la virgule flottante

Le code utilise des `float` (32 bits). Sur un STM32F4/F7 doté d'une FPU
matérielle, c'est gratuit — activer `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`. Sur un
STM32F103 (Cortex-M3, **sans** FPU), les opérations sont émulées : environ 2 à
3 µs pour l'interpolation complète des deux boucles, soit moins de 0.1 % d'un
cycle de 10 ms. Le passage en virgule fixe n'est donc **pas nécessaire** ; il ne
le deviendrait qu'en montant la fréquence de boucle au-delà du kilohertz.

---

## 11. Pièges rencontrés (et comment les éviter)

Ces erreurs ont réellement été rencontrées pendant le développement ; elles sont
documentées ici car elles se reproduiront lors du portage.

### 11.1 Signe inversé dans les fonctions d'appartenance

Le premier segment descendant des classes EG et TED s'écrivait `0.1·angle + 1`,
ce qui donne **-1** à -20° au lieu de +1. Les degrés d'appartenance devenaient
négatifs. L'inférence (qui prend des `max`) les ignorait silencieusement : aucune
erreur, mais les classes extrêmes ne s'activaient jamais correctement.

**Leçon** : toujours tracer les fonctions d'appartenance après les avoir écrites et
vérifier que µ ∈ [0, 1] partout. C'est le rôle de `fuzzy_1_fuzzification.png`.

### 11.2 Commande surdimensionnée

Avec des centres de sortie à ±12/±24, le robot sur-corrigeait si violemment que sa
vitesse angulaire sortait de l'univers du fuzzifier (±3 rad/s) — le contrôleur
devenait aveugle et le robot tombait en 0.6 s.

**Leçon** : la force réelle appliquée est `u/R` (soit ×30 sur le YahBoom). Les
centres de sortie doivent être calibrés sur la physique réelle, pas sur la borne
de saturation.

### 11.3 Zone morte de la ligne « Équilibre »

Voir §3.2 : « ne rien faire quand la vitesse est nulle » paraît naturel mais laisse
un robot penché sans correction.

**Leçon** : vérifier qu'aucune combinaison d'entrées plausible ne conduit à une
commande nulle alors qu'une action est nécessaire. La surface de commande
(`fuzzy_4_surface_commande.png`) rend ces plateaux immédiatement visibles.

### 11.4 Interférence entre les deux boucles

En réutilisant tels quels les gains du PID de position (`Kd = 0.1`) pour piloter la
boucle floue, le système divergeait en 1.15 s : le pic de dérivée saturait la cible
d'angle à ±10° en oscillation permanente.

**Leçon** : les gains d'une boucle externe ne sont **pas transférables** d'une
boucle interne à une autre. Il faut optimiser les deux boucles **ensemble** — ce
que fait l'optimiseur à 8 paramètres. Un `Kd` qui déstabilisait à la main est
devenu bénéfique une fois co-optimisé.

### 11.5 Boucle externe court-circuitée en auto-tuning

Dans `main_controller`, la branche PID désactive volontairement la boucle de
position quand `FAIRE_AUTOTUNING = True` (on veut alors régler l'angle seul). Ce
comportement avait été recopié pour la branche floue — alors que `FuzzyAutoTuner`
optimise les **deux** boucles ensemble.

Conséquence : avec `FAIRE_AUTOTUNING = True`, les paramètres optimaux étaient
appliqués à une architecture **amputée de sa boucle externe**. L'angle était tenu
parfaitement (0.02°) tandis que la position dérivait à plus de 3 m — un symptôme
trompeur, car le contrôleur semblait « fonctionner ».

**Leçon** : le code d'optimisation et le code d'exécution doivent simuler
**exactement** la même architecture. Toute divergence entre les deux produit des
paramètres optimaux pour un système qui n'est pas celui qui tourne.

---

*Projet EEIA26 — Étude comparative d'algorithmes de contrôle pour robot auto-balanceur*
