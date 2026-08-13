"""
export_actor_weights.py

Extrait uniquement l'acteur d'un modele SAC entrainé avec net_arch=[32,32].
La sortie du réseau est un duty cycle PWM normalisé dans [-1, 1].

Usage:
    python3 export_actor_weights.py sac_balance_curriculum.zip policy_sac.h
"""

import sys
import numpy as np
from stable_baselines3 import SAC


def format_array(name, arr, dims):
    flat = arr.flatten()
    lines = [f"static const float {name}{dims} = {{"]
    for i in range(0, len(flat), 8):
        chunk = flat[i:i+8]
        lines.append("    " + ", ".join(f"{v:.8f}f" for v in chunk) + ",")
    lines.append("};")
    return "\n".join(lines)


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 export_actor_weights.py <model.zip> <policy_sac.h>")
        sys.exit(1)

    model_path, out_path = sys.argv[1], sys.argv[2]
    model = SAC.load(model_path)
    actor = model.policy.actor

    # latent_pi: Sequential [Linear, ReLU, Linear, ReLU] pour net_arch=[32,32]
    layers = [m for m in actor.latent_pi if hasattr(m, "weight")]
    mu_layer = actor.mu  # derniere couche -> action (avant tanh, appliqué à l'inference)

    assert len(layers) == 2, f"Attendu 2 couches cachées, trouve {len(layers)} -- ajuste le script si net_arch a changé"

    W1 = layers[0].weight.detach().numpy()  # (H1, 4)
    B1 = layers[0].bias.detach().numpy()    # (H1,)
    W2 = layers[1].weight.detach().numpy()  # (H2, H1)
    B2 = layers[1].bias.detach().numpy()    # (H2,)
    W3 = mu_layer.weight.detach().numpy()   # (1, H2)
    B3 = mu_layer.bias.detach().numpy()     # (1,)

    H1, H2 = W1.shape[0], W2.shape[0]

    with open(out_path, "w") as f:
        f.write("#ifndef POLICY_SAC_H\n#define POLICY_SAC_H\n\n")
        f.write("// géneré par export_actor_weights.py -- !!ne pas editer à la main!!\n")
        f.write(f"#define SAC_H1 {H1}\n#define SAC_H2 {H2}\n")
        f.write("#define SAC_PWM_MAX 1.0f\n\n")
        f.write("// bornes de normalisation -- DOIVENT matcher obs_bounds dans balance_env.py\n")
        f.write("#define OBS_BOUND_X 0.5f\n#define OBS_BOUND_XDOT 2.0f\n")
        f.write("#define OBS_BOUND_THETA 0.7853981633974483f  // pi/4\n#define OBS_BOUND_THETADOT 5.0f\n\n")
        f.write(format_array("SAC_W1", W1, f"[{H1}][4]") + "\n\n")
        f.write(format_array("SAC_B1", B1, f"[{H1}]") + "\n\n")
        f.write(format_array("SAC_W2", W2, f"[{H2}][{H1}]") + "\n\n")
        f.write(format_array("SAC_B2", B2, f"[{H2}]") + "\n\n")
        f.write(format_array("SAC_W3", W3.flatten(), f"[{H2}]") + "\n\n")
        f.write(f"static const float SAC_B3 = {float(B3[0]):.8f}f;\n\n")
        f.write("#endif\n")

    total_params = W1.size + B1.size + W2.size + B2.size + W3.size + B3.size
    print(f"Ecrit {out_path}: {total_params} parametres (~{total_params*4/1024:.1f} KB)")


if __name__ == "__main__":
    main()
