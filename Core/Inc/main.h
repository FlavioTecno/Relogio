/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEGA_Pin GPIO_PIN_0
#define SEGA_GPIO_Port GPIOA
#define SEGB_Pin GPIO_PIN_1
#define SEGB_GPIO_Port GPIOA
#define SEGC_Pin GPIO_PIN_2
#define SEGC_GPIO_Port GPIOA
#define SEGD_Pin GPIO_PIN_3
#define SEGD_GPIO_Port GPIOA
#define SEGE_Pin GPIO_PIN_4
#define SEGE_GPIO_Port GPIOA
#define SEGF_Pin GPIO_PIN_5
#define SEGF_GPIO_Port GPIOA
#define SEGG_Pin GPIO_PIN_6
#define SEGG_GPIO_Port GPIOA
#define SEGP_Pin GPIO_PIN_7
#define SEGP_GPIO_Port GPIOA
#define DIG_1_Pin GPIO_PIN_10
#define DIG_1_GPIO_Port GPIOB
#define DIG_2_Pin GPIO_PIN_11
#define DIG_2_GPIO_Port GPIOB
#define DIG_3_Pin GPIO_PIN_12
#define DIG_3_GPIO_Port GPIOB
#define DIG_4_Pin GPIO_PIN_13
#define DIG_4_GPIO_Port GPIOB
#define DIG_LED_Pin GPIO_PIN_14
#define DIG_LED_GPIO_Port GPIOB
#define DIG_5_Pin GPIO_PIN_15
#define DIG_5_GPIO_Port GPIOB
#define DIG_6_Pin GPIO_PIN_8
#define DIG_6_GPIO_Port GPIOA
#define BUZAL_Pin GPIO_PIN_9
#define BUZAL_GPIO_Port GPIOA
#define LEDAL_Pin GPIO_PIN_10
#define LEDAL_GPIO_Port GPIOA
#define CH_HORA_Pin GPIO_PIN_11
#define CH_HORA_GPIO_Port GPIOA
#define CH_AL_Pin GPIO_PIN_12
#define CH_AL_GPIO_Port GPIOA
#define ALON_Pin GPIO_PIN_15
#define ALON_GPIO_Port GPIOA
#define BTUP_Pin GPIO_PIN_3
#define BTUP_GPIO_Port GPIOB
#define BTDOWN_Pin GPIO_PIN_4
#define BTDOWN_GPIO_Port GPIOB
#define BTESC_Pin GPIO_PIN_6
#define BTESC_GPIO_Port GPIOB
#define BTSET_Pin GPIO_PIN_7
#define BTSET_GPIO_Port GPIOB
#define LAMPAL_Pin GPIO_PIN_8
#define LAMPAL_GPIO_Port GPIOB
#define MOTORAL_Pin GPIO_PIN_9
#define MOTORAL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
