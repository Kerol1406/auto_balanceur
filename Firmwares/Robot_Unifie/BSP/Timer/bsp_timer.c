#include "bsp_timer.h"

/**
 * @file bsp_timer.c
 * @brief Gestionnaire de timer système (TIM6 - Interruption 10ms pour délai et batterie).
 */

static float battery_All = 0.0f;
static uint8_t battery_count = 0, battery_flag = 0;
static u16 stop_time = 0;

void delay_time(u16 time)
{
    stop_time = time;
    while (stop_time);
}

void my_delay(u16 s)
{
    for (int i = 0; i < s; i++)
    {
        delay_time(100);
    }
}

void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 100 - 1; // Interruption 10ms
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        battery_flag++;

        if (stop_time > 0)
        {
            stop_time--;
        }

        // Échantillonnage et moyenne de la tension batterie toutes les 1000ms
        if (battery_flag > 2) // 20ms
        {
            battery_flag = 0;
            battery_All += Get_Battery_Volotage();
            battery_count++;
            if (battery_count == 50) // 1000ms
            {
                battery = battery_All / 50.0f;
                battery_All = 0.0f;
                battery_count = 0;
            }
        }
    }
}
