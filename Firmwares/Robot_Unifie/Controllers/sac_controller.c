#include "policy_sac.h"
#include "controller_sac.h"
#include <math.h>

#if TYPE_CONTROLLER == 3



 float relu(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

float Controller_Compute(RobotState state)
{
    // --- Normalisation de l'observation, MEME ORDRE que l'entrainement Python ---
    // obs = [x, x_dot, theta, theta_dot] / obs_bounds
    float obs[4];
    obs[0] = state.x        / OBS_BOUND_X;
    obs[1] = state.x_dot    / OBS_BOUND_XDOT;
    obs[2] = state.theta    / OBS_BOUND_THETA;
    obs[3] = state.theta_dot/ OBS_BOUND_THETADOT;

    // --- Couche 1 : Linear(4 -> H1) + ReLU ---
    float h1[SAC_H1];
    for (int i = 0; i < SAC_H1; i++) {
        float acc = SAC_B1[i];
        for (int j = 0; j < 4; j++) {
            acc += SAC_W1[i][j] * obs[j];
        }
        h1[i] = relu(acc);
    }

    // --- Couche 2 : Linear(H1 -> H2) + ReLU ---
    float h2[SAC_H2];
    for (int i = 0; i < SAC_H2; i++) {
        float acc = SAC_B2[i];
        for (int j = 0; j < SAC_H1; j++) {
            acc += SAC_W2[i][j] * h1[j];
        }
        h2[i] = relu(acc);
    }

    // --- Couche de sortie : Linear(H2 -> 1), puis tanh ---
    float pre_tanh = SAC_B3;
    for (int j = 0; j < SAC_H2; j++) {
        pre_tanh += SAC_W3[j] * h2[j];
    }
    float action = tanhf(pre_tanh);   // dans [-1, 1]


    return action;
}



#endif /* TYPE_CONTROLLER == 3 */

