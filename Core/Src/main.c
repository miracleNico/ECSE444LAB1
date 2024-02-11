/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stats.h"
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {

	float q;
	float p;
	float r;
	float k;
	float x;
} Karman;

extern int updateKarmanFilter_ASM(Karman *kr, float m);
extern int updateKarmanFilter_c(Karman *kr, float m);
extern int updateKarmanFilter_CMSIS(Karman *kr, float m);
//extern int Karman_init(Karman* kr);

void KalmanFilter(float *InputArray, float *OutputArray, Karman *kr, int len) {
	for (int i = 0; i < len; i++) {
		if (updateKarmanFilter_ASM(kr, *(InputArray + i))) {
			Error_Handler();
		}
		;
		//updateKarmanFilter_c(kr, InputArray[i]);
		//updateKarmanFilter_CMSIS(kr, InputArray[i]);
		*(OutputArray + i) = kr->x;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int clk = HAL_RCC_GetSysClockFreq();
	int _write(int file, char *ptr, int len)
	{
	  /* Implement your write code here, this is used by puts and printf for example */
		int i=0;
		for(i=0 ; i<len ; i++)
			ITM_SendChar((*ptr++));
		return len;
	}

	void print_csv_SWV(float* arr,int len){
		for(int i=0; i<len; i++){
			printf("%f,",arr[i]);
		}

	}

	Karman kr;
	kr.q = 0.1;
	kr.p = 0.1;
	kr.r = 0.1;
	kr.k = 0.0;
	kr.x = 5.0;

	float inputArray[] = { 10.4915760032, 10.1349974709, 9.53992591829,
			9.60311878706, 10.4858891793, 10.1104642352, 9.51066931906,
			9.75755656493, 9.82154078273, 10.2906541933, 10.4861328671,
			9.57321181356, 9.70882714139, 10.4359069357, 9.70644021369,
			10.2709894039, 10.0823149505, 10.2954563443, 9.57130449017,
			9.66832136479, 10.4521677502, 10.4287240667, 10.1833650752,
			10.0066049721, 10.3279461634, 10.4767210803, 10.3790964606,
			10.1937408814, 10.0318963522, 10.4939180917, 10.2381858895,
			9.59703103024, 9.62757986516, 10.1816981174, 9.65703773168,
			10.3905666599, 10.0941977598, 9.93515274393, 9.71017053437,
			10.0303874259, 10.0173504397, 9.69022731474, 9.73902896102,
			9.52524419732, 10.3270730526, 9.54695650657, 10.3573960542,
			9.88773266876, 10.1685038683, 10.1683694089, 9.88406620159,
			10.3290065898, 10.2547227265, 10.4733422906, 10.0133952458,
			10.4205693583, 9.71335255372, 9.89061396699, 10.1652744131,
			10.2580948608, 10.3465431058, 9.98446410493, 9.79376005657,
			10.202518901, 9.83867150985, 9.89532986869, 10.2885062658,
			9.97748768804, 10.0403923759, 10.1538911808, 9.78303667556,
			9.72420149909, 9.59117495073, 10.1716116012, 10.2015818969,
			9.90650056596, 10.3251329834, 10.4550120431, 10.4925749165,
			10.1548177178, 9.60547133785, 10.4644672766, 10.2326496615,
			10.2279703226, 10.3535284606, 10.2437410625, 10.3851531317,
			9.90784804928, 9.98208344925, 9.52778805729, 9.69323876912,
			9.92987312087, 9.73938925207, 9.60543743477, 9.79600805462,
			10.4950988486, 10.2814361401, 9.7985283333, 9.6287888922,
			10.4491538991, 9.5799256668 };
	int len = sizeof(inputArray) / sizeof(float);
	float outputArray[len];
	float diffArray[len];
	float convArray[2 * len - 1];
	float average, average_CMSIS, std, std_CMSIS, corr, corr_CMSIS;

	while (1) {
		ITM_Port32(31) = 1;

		KalmanFilter(inputArray, outputArray, &kr, len);
		ITM_Port32(31) = 2;

		calculateDiff(inputArray, outputArray, diffArray, len);
		ITM_Port32(31) = 3;

		calculateDiff_CMSIS(inputArray, outputArray, diffArray, len);
		ITM_Port32(31) = 4;

		average = calculateAverage(diffArray, len);
		ITM_Port32(31) = 5;

		average_CMSIS = calculateAverage_CMSIS(diffArray, len);
		ITM_Port32(31) = 6;

		std = calculateStd(outputArray, len);
		ITM_Port32(31) = 7;

		std_CMSIS = calculateStd_CMSIS(outputArray, len);
		ITM_Port32(31) = 8;

		calculateConv(inputArray, outputArray, convArray, len);
		ITM_Port32(31) = 9;

		calculateConv_CMSIS(inputArray, outputArray, convArray, len);
		ITM_Port32(31) = 10;

		corr = calculateCorr(inputArray, outputArray, len);
		ITM_Port32(31) = 11;

		corr_CMSIS = calculateCorr_CMSIS(inputArray, outputArray, len);
		ITM_Port32(31) = 12;

		print_csv_SWV(inputArray, len);

		print_csv_SWV(outputArray, len);

		HAL_Delay(1000);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
