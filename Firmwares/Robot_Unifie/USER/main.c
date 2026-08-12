#include "AllHeader.h"
#include "intsever.h"

/**
 * @file main.c
 * @brief Point d'entrée principal du projet unifié Robot_Unifie.
 */

uint8_t GET_Angle_Way = 2;              // Algorithme d'attitude : 1 = DMP, 2 = Kalman, 3 = Complémentaire
float Angle_Balance = 0.0f;             // Angle d'inclinaison pitch (degrés)
float Gyro_Balance = 0.0f;              // Vitesse angulaire gyro (degrés/s)
int Motor_Left = 0, Motor_Right = 0;    // Commandes PWM moteurs
float Move_X = 0.0f, Move_Z = 0.0f;     // Vitesse de consigne (avance/rotation)

u8 Stop_Flag = 0;                       // Drapeaux de sécurité : 0 = En marche (démarrage direct)

char showbuf[30] = {'\0'};

int main(void)
{
    // Initialisation matérielle globale (BSP)
    bsp_init();

    // Initialisation de l'interruption matérielle MPU6050 (PA12)
    MPU6050_EXTI_Init();

    printf("=== Robot_Unifie Initialise ===\r\n");

#if TYPE_CONTROLLER == 2
    printf("Controleur Actif : PID (Double boucle PD/PI)\r\n");
    OLED_Draw_Line("Mode: PID (PD/PI)", 1, true, true);
#elif TYPE_CONTROLLER == 1
    printf("Controleur Actif : LQR (Retour d'etat 4D)\r\n");
    OLED_Draw_Line("Mode: LQR (4D)", 1, true, true);
#elif TYPE_CONTROLLER == 3
    printf("Controleur Actif : RL-SAC \r\n");
    OLED_Draw_Line("Mode: RL-SAC", 1, true, true);
#endif

    Stop_Flag = 0; // Démarrage du contrôle moteur direct (identique à LQR_Controller)

    OLED_Draw_Line("Controle Actif!", 2, false, true);

    // Boucle principale (Supervision et affichage OLED)
    while (1)
    {
        // Appui sur Key1 pour basculer Marche / Arrêt
        if (Key1_State(1) == KEY_PRESS)
        {
            Stop_Flag = !Stop_Flag;
            delay_ms(200);
        }

        // Affichage de l'angle d'inclinaison sur l'OLED
        sprintf(showbuf, "Angle : %.2f deg ", Angle_Balance);
        OLED_Draw_Line(showbuf, 3, false, true);

        // Affichage de la tension batterie
        sprintf(showbuf, "Bat   : %.2f V   ", battery);
        OLED_Draw_Line(showbuf, 4, false, true);

        delay_ms(100);
    }
}
