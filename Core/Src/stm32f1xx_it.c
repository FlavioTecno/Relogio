/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "menu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern const uint8_t font[18];
extern uint8_t buffer[5];
extern int testeok;

#if ComSeg
extern unsigned char buffer_diplay;
#else
extern unsigned char buffer_diplay[] = { '8', '8', '8', '8', 0x00};
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC global interrupt.
  */
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */

  /* USER CODE END RTC_IRQn 0 */
  HAL_RTCEx_RTCIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */

  /* USER CODE END RTC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  static uint8_t digit=0;


#if !(Relogio)
  if (testeok == 0){
#else
  if (testeok == 1){
#endif
	  HAL_GPIO_TogglePin(DIG_LED_GPIO_Port, DIG_LED_Pin);
	  HAL_GPIO_TogglePin(DIG_1_GPIO_Port, DIG_1_Pin);
	  HAL_GPIO_TogglePin(DIG_2_GPIO_Port, DIG_2_Pin);
	  HAL_GPIO_TogglePin(DIG_3_GPIO_Port, DIG_3_Pin);
	  HAL_GPIO_TogglePin(DIG_4_GPIO_Port, DIG_4_Pin);
#if ComSeg
	  HAL_GPIO_TogglePin(DIG_5_GPIO_Port, DIG_5_Pin);
	  HAL_GPIO_TogglePin(DIG_6_GPIO_Port, DIG_6_Pin);
#endif
  }
  else {
	  	  GPIOA -> ODR = buffer[digit];

	  	  switch(digit){
#if TipoDisp
		  case 0:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
#if ComSeg
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
#endif
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
			  break;
		  case 1:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
#if ComSeg
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
#endif
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
			  break;
		  case 2:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
#if ComSeg
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
#endif
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
			  break;
		  case 3:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_SET);
#if ComSeg
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
#endif
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
			  break;
		  case 4:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
#if ComSeg
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
#else
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_SET);
#endif
			  break;
#if ComSeg
		  case 5:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_RESET);
			  break;
		  case 6:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_5_GPIO_Port, DIG_5_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_6_GPIO_Port, DIG_6_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_LED_GPIO_Port, DIG_LED_Pin, GPIO_PIN_SET);
			  break;
#endif

#else	//anodo comum
		  case 0:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_RESET);
			  break;
		  case 1:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_SET);
			  break;
		  case 2:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_SET);
			  break;
		  case 3:
			  HAL_GPIO_WritePin(DIG_1_GPIO_Port, DIG_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_2_GPIO_Port, DIG_2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_3_GPIO_Port, DIG_3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIG_4_GPIO_Port, DIG_4_Pin, GPIO_PIN_SET);
			  break;
#endif
		}

		digit++;
		if (digit > (SS_DIGIT - 1)){
		  digit = 0;
		}
  }

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
