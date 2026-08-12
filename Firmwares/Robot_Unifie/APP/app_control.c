#include "app_control.h"

/**
 * @file app_control.c
 * @brief Orchestration globale de la boucle d'asservissement temps réel (200 Hz).
 * 
 * Ce fichier gère l'interruption matérielle du MPU6050, l'acquisition des capteurs,
 * la mise à jour de la structure d'état RobotState et le pilotage des moteurs via
 * la méthode de contrôle sélectionnée dans controller_config.h (LQR ou PID).
 */

// Données d'état du robot
float x_pose = 0.0f;       // Position linéaire x (m)
float x_speed = 0.0f;      // Vitesse linéaire dx/dt (m/s)
float angle_x = 0.0f;      // Angle d'inclinaison (rad)
float gyro_x = 0.0f;       // Vitesse angulaire d'inclinaison (rad/s)
float last_angle = 0.0f;   // Dernier angle d'inclinaison enregistré

int velocity_L = 0;        // Commande vitesse brute moteur gauche
int velocity_R = 0;        // Commande vitesse brute moteur droit

float Ratio_accel = 2400.0f; // Coefficient de conversion accélération -> PWM
float PWM_MAX = 2600;
float battery = 12.0f;       // Tension batterie (12V par défaut)
float Acceleration_Z = 0.0f; // Accélération axe Z
int Mid_Angle = 0;           // Médiane mécanique (0°)


/**************************************************************************
 * Fonction : EXTI15_10_IRQHandler (200 Hz / Période de 5 ms)
 * Interruption matérielle du MPU6050 sur la broche PA12.
 **************************************************************************/
void EXTI15_10_IRQHandler(void)
{
    int Encoder_Left, Encoder_Right;

    if (MPU6050_INT == 0)
    {
        // 1. Acquittement du drapeau d'interruption EXTI 12
        EXTI->PR = 1 << 12;
        (void)EXTI->PR;

        // 2. Acquisition de l'angle d'inclinaison (DMP / Kalman / Complémentaire)
        Get_Angle(GET_Angle_Way);

        // 3. Lecture des encodeurs de roues (Sens positif vers l'avant)
        Encoder_Left = Read_Encoder(MOTOR_ID_ML);
        Encoder_Right = -Read_Encoder(MOTOR_ID_MR);
        Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

        // Calcul de la vitesse linéaire (m/s) et intégration de la position (m)
        x_speed = (float)(Encoder_Left + Encoder_Right) / 2.0f * PI * Diameter_67 / 1000.0f / 1560.0f * Control_Frequency;
        x_pose += x_speed / Control_Frequency;

        // Conversion de l'angle d'inclinaison (deg -> rad) et vitesse angulaire (rad/s)
        angle_x = Angle_Balance / 180.0f * PI;
        gyro_x = (angle_x - last_angle) * Control_Frequency;
        last_angle = angle_x;

        // 4. Construction du vecteur d'état du robot
        RobotState state;
        state.x = x_pose;
        state.x_dot = x_speed;
        state.theta = angle_x;
#if TYPE_CONTROLLER == 2
        state.theta_dot = Gyro_Balance; // Transmet Gyro_Balance brut au PD
#else
        state.theta_dot = gyro_x;
#endif


        // 5. Calcul de la commande via l'interface unifiée Controller_Compute
        float control_output = Controller_Compute(state);

#if TYPE_CONTROLLER == 1
        // Mode LQR : control_output est l'accélération demandée (m/s²)
        velocity_L = (int)(Ratio_accel * (x_speed + control_output / Control_Frequency));
        velocity_R = velocity_L;

#elif TYPE_CONTROLLER == 2
        // Mode PID : control_output est la composante PWM d'équilibre PD
        int balance_pwm = (int)control_output;
        int velocity_pwm = Velocity_PI(Encoder_Left, Encoder_Right);
        velocity_L = balance_pwm + velocity_pwm;
        velocity_R = balance_pwm + velocity_pwm;

#elif TYPE_CONTROLLER == 3
        // Mode RL : control_output est le duty cycle normalisé
        velocity_L = (float) control_output * PWM_MAX ;
        velocity_R = velocity_L;

#endif

        // 6. Filtrage des zones mortes et limitation PWM dans la plage [-2600, 2600]
        Motor_Left = PWM_Limit(PWM_Ignore(velocity_L), 2600, -2600);
        Motor_Right = PWM_Limit(PWM_Ignore(velocity_R), 2600, -2600);

        // 7. Envoi de la commande aux registres PWM si la sécurité batterie/inclinaison est valide
        if (Turn_Off(Angle_Balance, battery) == 0 && Stop_Flag == 0)
        {
            Set_Pwm(Motor_Left, Motor_Right);
        }
        else
        {
            Set_Pwm(0, 0);
        }
    }
}

/********************************==========================================
 * Fonction : Get_Angle
 * Permet d'acquérir l'attitude du robot selon l'algorithme sélectionné.
 **************************************************************************/
void Get_Angle(u8 way)
{
    float gyro_x_val, gyro_y_val, gyro_z_val;
    float accel_x_val, accel_y_val, accel_z_val;
    float Accel_Angle_x, Accel_Angle_y;

    if (way == 1) // Lecture DMP directe
    {
        Read_DMP();
        Angle_Balance = Pitch;
        Gyro_Balance = gyro[0];
        Acceleration_Z = accel[2];
    }
    else
    {
        // Lecture I2C Burst rapide
        Read_MPU6050_Burst(&gyro_x_val, &gyro_y_val, &gyro_z_val,
                           &accel_x_val, &accel_y_val, &accel_z_val);

        if (GET_Angle_Way == 2) // Filtre de Kalman
        {
            Pitch = KF_X(accel_y_val, accel_z_val, -gyro_x_val) / PI * 180.0f;
            Roll  = KF_Y(accel_x_val, accel_z_val, gyro_y_val) / PI * 180.0f;
        }
        else if (GET_Angle_Way == 3) // Filtre Complémentaire
        {
            Accel_Angle_x = atan2(accel_y_val, accel_z_val) * 180.0f / PI;
            Accel_Angle_y = atan2(accel_x_val, accel_z_val) * 180.0f / PI;

            Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x_val);
            Roll  = -Complementary_Filter_y(Accel_Angle_y, gyro_y_val);
        }
        Angle_Balance = Pitch;
    }
}

/********************************==========================================
 * Fonction : Read_MPU6050_Burst
 * Lecture I2C burst accélérée (14 octets en 1 seule transaction I2C).
 **************************************************************************/
void Read_MPU6050_Burst(float *gyro_x_val, float *gyro_y_val, float *gyro_z_val,
                         float *accel_x_val, float *accel_y_val, float *accel_z_val)
{
    uint8_t buffer[14];
    int16_t Gyro_X, Gyro_Y, Gyro_Z;
    int16_t Accel_X, Accel_Y, Accel_Z;

    // Lecture BURST des 14 registres du MPU6050
    IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);

    // Extraction et reconstitution 16-bit
    Accel_X = (buffer[0] << 8) | buffer[1];
    Accel_Y = (buffer[2] << 8) | buffer[3];
    Accel_Z = (buffer[4] << 8) | buffer[5];
    Gyro_X  = (buffer[8] << 8) | buffer[9];
    Gyro_Y  = (buffer[10] << 8) | buffer[11];
    Gyro_Z  = (buffer[12] << 8) | buffer[13];

    // Conversion signée
    if (Gyro_X > 32768)  Gyro_X -= 65536;
    if (Gyro_Y > 32768)  Gyro_Y -= 65536;
    if (Gyro_Z > 32768)  Gyro_Z -= 65536;
    if (Accel_X > 32768) Accel_X -= 65536;
    if (Accel_Y > 32768) Accel_Y -= 65536;
    if (Accel_Z > 32768) Accel_Z -= 65536;

    // Conversion en unités physiques
    *accel_x_val = Accel_X / 1671.84f;
    *accel_y_val = Accel_Y / 1671.84f;
    *accel_z_val = Accel_Z / 1671.84f;
    *gyro_x_val  = Gyro_X / 939.8f;
    *gyro_y_val  = Gyro_Y / 939.8f;
    *gyro_z_val  = Gyro_Z / 939.8f;

    Gyro_Balance   = -Gyro_X;
    Acceleration_Z = Accel_Z;
}
