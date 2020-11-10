/* USER CODE END Header */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TipoDisp	1	//1 -> Catodo
						//0	-> Anodo

#define ComSeg		1	//1 -> Com Segundos
						//0 -> sem Segundos

#if	!ComSeg
#define SS_DIGIT	6	//sem segundos
#else
#define SS_DIGIT	8	//com segundos
#endif

#define Relogio		1	//1 = função relogio
						//0 = teste de dispositivo

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
