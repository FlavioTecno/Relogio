/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * Atualização
  * 23/10/2020 - Implementado relogio com segundpos
  * 10/11/2020 - Implementado botões do menu
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "menu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef clkTime;
RTC_DateTypeDef clkDate;

uint8_t Horas, Minutos, Segundos;

int Dez_Hora=0, Uni_Hora=0, Dez_Minuto=0, Uni_Minuto=0, Dez_Segundos=0, Uni_Segundos=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*----------------------------------------------------------
Configura��o do display de 7 segmentos e pinos de conex�o
     A
    ----
  F|    |B
   |  G |
    ----
  E|    |C
   |    |
    ---- *p
      D
Segmento A - Conectado pino 15 do 595
Segmento B - Conectado pino 1 do 595
Segmento C - Conectado pino 2 do 595
Segmento D - Conectado pino 3 do 595
Segmento E - Conectado pino 4 do 595
Segmento F - Conectado pino 5 do 595
Segmento G - Conectado pino 6 do 595
Segmento p - Conectado pino 7 do 595

Sequencia do byte: A B C D E F G p

Essa codifica vai de 0 a 9. Mas varias letras
pode ser desenhadas, assim como outros caracteres  básicos
------------------------------------------------------------*/

const unsigned char font[22] =
{
	0x3F,    /* 0 */
	0x06,    /* 1 */
	0x5B,    /* 2 */
	0x4F,    /* 3 */
	0x66,    /* 4 */
	0x6D,    /* 5 */
	0x7D,    /* 6 */
	0x07,    /* 7 */
	0x7F,    /* 8 */
	0x6F,    /* 9 */
	0x77,    /* A */
	0x7C,    /* B */
	0x39,    /* C */
	0x5E,    /* D */
	0x79,    /* E */
	0x71,    /* F */
	0x37,	/* M */
	0x39,	/* E */
	0x54,	/* N */
	0x1C,	/* U */
	0X80,	/* Ponto */
	0x00	/* Apagado */
};

const uint8_t Segmentos[8]=
{
	0x01,	//seg A
	0x02,	//seg B
	0x04,	//seg C
	0x08,	//seg D
	0x10,	//seg E
	0x20,	//seg F
	0x40,	//seg G
	0x80	//seg P
};

#if ComSeg
uint8_t buffer[7];
#else
uint8_t buffer[4];
#endif

int curDigit = 0;
int cont;
int testeok;
int BotSet, BotInc, BotDec;


void PrintNumber(uint32_t number)
{
	int conver;
#if !ComSeg
    // Check max and min
    if (number > 9999)
    {
        number = 9999;
    }

    // Convert integer to bcd digits

    buffer[4] = 0x80;

    conver = number / 1000;
    buffer[3] = font[conver];
    conver = number % 1000/100;
    buffer[2] = font[conver];
    conver = number % 100/10;
    buffer[1] = font[conver];
    conver = number % 10;
    buffer[0] = font[conver];
#else
    // Check max and min
    if (number > 999999)
    {
        number = 999999;
    }

    // Convert integer to bcd digits

    buffer[6] = 0x80;

    conver = number / 100000;
    buffer[5] = font[conver];
    conver = number % 100000/10000;
    buffer[4] = font[conver];
    conver = number % 10000/1000;
    buffer[3] = font[conver];
    conver = number % 1000/100;
    buffer[2] = font[conver];
    conver = number % 100/10;
    buffer[1] = font[conver];
    conver = number % 10;
    buffer[0] = font[conver];
#endif
}	//end PrintNumber


void PrintRelogio(uint8_t tempo)
{
#if !ComSeg
	Dez_Hora = Horas / 10;
	buffer[3] = font[Dez_Hora];
	Uni_Hora = Horas % 10;
	buffer[2] = font[Uni_Hora];
	Dez_Minuto = Minutos /10;
	buffer[1] = font[Dez_Minuto];
	Uni_Minuto = Minutos % 10;
	buffer[0] = font[Uni_Minuto];
#else
	Dez_Hora = Horas / 10;
	buffer[5] = font[Dez_Hora];
	Uni_Hora = Horas % 10;
	buffer[4] = font[Uni_Hora];
	Dez_Minuto = Minutos /10;
	buffer[3] = font[Dez_Minuto];
	Uni_Minuto = Minutos % 10;
	buffer[2] = font[Uni_Minuto];
	Dez_Segundos = Segundos /10;
	buffer[1] = font[Dez_Segundos];
	Uni_Segundos = Segundos % 10;
	buffer[0] = font[Uni_Segundos];
#endif
}

void MenuRelogio(void){
	while (BotSet == 0) {
		buffer[5] = 0x37;
		buffer[4] = 0x79;
		buffer[3] = 0X54;
		buffer[2] = 0X1C;
		buffer[1] = 0x00;
		buffer[0] = 0x00;

		BotInc = HAL_GPIO_ReadPin(Bot_Inc_GPIO_Port, Bot_Inc_Pin);
		if (BotInc == 0){
			HAL_Delay(20);
			buffer[6] = 0x80;	//liga leds
			BotInc = 1;
			BotSet = 1;
		}

		BotDec = HAL_GPIO_ReadPin(Bot_Dec_GPIO_Port, Bot_Dec_Pin);
		if (BotDec == 0) {
			HAL_Delay(20);
			buffer[6] = 0x00;	//desliga leds
		    BotDec = 1;
		    BotSet = 1;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //inicializa timer 2
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_RTC_Init(&hrtc);
  HAL_RTCEx_SetSecond_IT(&hrtc);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  BotSet = 1;
	  BotInc = 1;
	  BotDec = 1;

#if !Relogio
	  testeok = HAL_GPIO_ReadPin(Teste_GPIO_Port, Teste_Pin);

	  if (testeok == 1)
		  for (cont = 0; cont <= 7; cont++){
			  GPIOA -> ODR = Segmentos[cont];
			  HAL_Delay(250);
		  }
	  else {
#if !ComSeg
		for (cont = 0; cont < 10000; cont++) {
#else
		for (cont = 0; cont < 100000; cont++){
#endif
			PrintNumber(cont);
			HAL_Delay(500);
			testeok = HAL_GPIO_ReadPin(Teste_GPIO_Port, Teste_Pin);
			if (testeok == 1) break;
		}
	  }

#if !ComSeg
	  PrintNumber(2175);
	  HAL_Delay(500);

	  PrintNumber(7520);
	  HAL_Delay(500);
#else
	  PrintNumber(524778);
	  HAL_Delay(500);

	  PrintNumber(343074);
	  HAL_Delay(500);
#endif

#else

	  if(HAL_GPIO_ReadPin(Bot_Set_GPIO_Port, Bot_Set_Pin) == GPIO_PIN_RESET){			  //Verifica botao menu
		  BotSet = 0;
		  MenuRelogio();
	  }


	  HAL_RTC_GetTime(&hrtc, &clkTime, RTC_FORMAT_BIN);

	  Horas = clkTime.Hours;
	  Minutos = clkTime.Minutes;
#if ComSeg
	  Segundos = clkTime.Seconds;
	  PrintRelogio(Segundos);
#endif
	  PrintRelogio(Minutos);
	  PrintRelogio(Horas);
#endif	//relogio

  }		//end while
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TesteLed_GPIO_Port, TesteLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin
                          |SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGP_Pin
                          |DIG_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIG_1_Pin|DIG_2_Pin|DIG_3_Pin|DIG_4_Pin
                          |DIG_LED_Pin|DIG_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TesteLed_Pin */
  GPIO_InitStruct.Pin = TesteLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TesteLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEGA_Pin SEGB_Pin SEGC_Pin SEGD_Pin
                           SEGE_Pin SEGF_Pin SEGG_Pin SEGP_Pin
                           DIG_6_Pin */
  GPIO_InitStruct.Pin = SEGA_Pin|SEGB_Pin|SEGC_Pin|SEGD_Pin
                          |SEGE_Pin|SEGF_Pin|SEGG_Pin|SEGP_Pin
                          |DIG_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG_1_Pin DIG_2_Pin DIG_3_Pin DIG_4_Pin
                           DIG_LED_Pin DIG_5_Pin */
  GPIO_InitStruct.Pin = DIG_1_Pin|DIG_2_Pin|DIG_3_Pin|DIG_4_Pin
                          |DIG_LED_Pin|DIG_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Bot_Set_Pin Bot_Inc_Pin Bot_Dec_Pin */
  GPIO_InitStruct.Pin = Bot_Set_Pin|Bot_Inc_Pin|Bot_Dec_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
