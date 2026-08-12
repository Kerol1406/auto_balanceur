/**
* @file         AllHeader.h
* @brief        Inclusions globales pour le projet unifié Robot_Unifie
*/

#ifndef __ALLHEADER_H
#define __ALLHEADER_H

// Bibliothèques C standards
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// En-têtes STM32
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#include "switch_function.h"
#include "myenum.h"

// En-têtes BSP
#include "delay.h"
#include "bsp.h"
#include "bsp_battery.h"
#include "bsp_beep.h"
#include "bsp_LED.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "usart.h"
#include "bsp_oled.h"
#include "bsp_oled_i2c.h"

// En-têtes MPU6050 & Capteurs
#include "IOI2C.h"
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

// En-têtes Moteurs & Encodeurs
#include "motor.h"
#include "encoder.h"
#include "app_motor.h"

// En-têtes Application & Filtrage
#include "app_control.h"
#include "filter.h"
#include "KF.h"

// En-têtes Contrôleurs (Unifié LQR / PID)
#include "controller_config.h"
#include "controller_common.h"
#if TYPE_CONTROLLER == 1
  #include "controller_lqr.h"
#elif TYPE_CONTROLLER == 2
  #include "controller_pid.h"
#elif TYPE_CONTROLLER == 3
  #include "controller_sac.h"
#endif

#endif /* __ALLHEADER_H */
