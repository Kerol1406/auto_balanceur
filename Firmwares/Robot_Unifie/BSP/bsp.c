#include "bsp.h"
#include "intsever.h"

/**
 * @file bsp.c
 * @brief Initialisation globale de la couche d'abstraction matérielle (BSP).
 */

void bsp_init(void)
{
    // 1. Définition du groupe de priorités NVIC
    DIY_NVIC_PriorityGroupConfig(2);

    // 2. Initialisation des délais système
    delay_init();

    // 3. Configuration de l'interface de débogage JTAG / SWD
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    // 4. Initialisation des I/O de base (LED, Buzzer, Touches)
    init_led_gpio();
    init_beep();
    Key1_GPIO_Init();

    // 5. Initialisation de la motorisation et des encodeurs
    BalanceCar_Motor_Init();
    BalanceCar_PWM_Init(2880, 0); // PWM 25 kHz
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();

    // 6. Initialisation des communications série
    uart_init(115200);

    delay_ms(300);

    // 7. Initialisation du capteur MPU6050
    IIC_MPU6050_Init();
    MPU6050_initialize();
    DMP_Init();

    // 8. Initialisation de l'affichage OLED et du test batterie
    OLED_I2C_Init();
    Battery_init();

    // 9. Initialisation du Timer 6 (Service 10 ms pour la batterie et le clignotement)
    TIM6_Init();
}

void JTAG_Set(u8 mode)
{
    u32 temp;
    temp = mode;
    temp <<= 25;
    RCC->APB2ENR |= 1 << 0;     // Active l'horloge AFIO
    AFIO->MAPR &= 0XF8FFFFFF;   // Efface les bits 26-24
    AFIO->MAPR |= temp;         // Applique le mode JTAG/SWD
}

void DIY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
    u32 temp, temp1;
    temp1 = (~NVIC_Group) & 0x07; // Masque 3 bits
    temp1 <<= 8;
    temp = SCB->AIRCR;
    temp &= 0X0000F8FF;           // Efface le registre AIRCR
    temp |= 0X05FA0000;           // Clé d'écriture
    temp |= temp1;
    SCB->AIRCR = temp;            // Écriture de la nouvelle configuration
}
