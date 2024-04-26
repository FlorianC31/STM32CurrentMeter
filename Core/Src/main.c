/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NB_ADC_PINS 7

// ADC
#define VCC 			3.3
#define BITS_ADC 		12

// Number of samples on a half-cycle
#define SAMPLES			100

#define LUT_SIZE		100

// Sensors
#define CURR_COEFF 		2000
#define TRANSF_MAIN 	230
#define TRANSF_SECOND 	6

// Resistors
#define R10				10
#define R100			100
#define R220			220
#define R1K				1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int adcConversionComplete = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
float ratioFromLUT(float* dutyRatioLut, float inputRatio);
void initLUT(float* dutyRatioLut);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t data [NB_ADC_PINS];
	int nLoop = 0;
	float dutyRatioLut[LUT_SIZE+1];

	// Coeff definition
	double globalCoeff = VCC / (pow(2, BITS_ADC) - 1);		// From 12bits digital value[0-4095] to ADC tension range [0-3.3V]
	double coeff[NB_ADC_PINS];

	// Current sensors
	coeff[0] = globalCoeff * CURR_COEFF * (1/R220 + R100 + R100);	// Current Detection Range Icc(A):   [-81;81] / Max Power Peff(kW): 13.2 (PAC)
	coeff[1] = globalCoeff * CURR_COEFF / R100;						// Current Detection Range Icc(A):   [-33;33] / Max Power Peff(kW):  5.4 (Water Heater)
	coeff[2] = globalCoeff * CURR_COEFF / R100;						// Current Detection Range Icc(A):   [-33;33] / Max Power Peff(kW):  5.4 (Water Pump)
	coeff[3] = globalCoeff * CURR_COEFF * (1/R220 + 1/R100);		// Current Detection Range Icc(A):   [-48;48] / Max Power Peff(kW):  7.8 (Prod PV)
	coeff[4] = globalCoeff * CURR_COEFF / (R10 + R10 + R10);		// Current Detection Range Icc(A): [-110;110] / Max Power Peff(kW): 17.9 (General)

	// Tension sensor - Tension Detection Range Ucc(V): [-351;351] / Max Tension Ueff(V): 248 V
	coeff[5] = globalCoeff * (R1K + R220) / R220 * TRANSF_MAIN / TRANSF_SECOND;

	// VRef sensor - Tension Detection Range Udc(V): [0-3.3]
	coeff[6] = globalCoeff;

	//initLUT(dutyRatioLut);


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  //MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, NB_ADC_PINS);
	HAL_ADC_Start_DMA(&hadc1, data, NB_ADC_PINS);
	nLoop = 0;
	while (adcConversionComplete == 0 && nLoop < 1000){
		nLoop++;
	}
	adcConversionComplete = 0;
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL14;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcConversionComplete = 1;
}

float ratioFromLUT(float* dutyRatioLut, float inputRatio)
{
	return dutyRatioLut[(int)(inputRatio * LUT_SIZE)];
}

void initLUT(float* dutyRatioLut)
{
	dutyRatioLut[0]   = 0.0;
	dutyRatioLut[1]   = 0.0201117939656107;
	dutyRatioLut[2]   = 0.0325028736698596;
	dutyRatioLut[3]   = 0.0429145335877155;
	dutyRatioLut[4]   = 0.0521950973510767;
	dutyRatioLut[5]   = 0.0607393439205089;
	dutyRatioLut[6]   = 0.0687643178339433;
	dutyRatioLut[7]   = 0.0763665366772231;
	dutyRatioLut[8]   = 0.0836457101808669;
	dutyRatioLut[9]   = 0.0906206416839455;
	dutyRatioLut[10]  = 0.0973758763906862;
	dutyRatioLut[11]  = 0.103907277521487;
	dutyRatioLut[12]  = 0.110274071682577;
	dutyRatioLut[13]  = 0.116482887098396;
	dutyRatioLut[14]  = 0.122550835950779;
	dutyRatioLut[15]  = 0.128492499519223;
	dutyRatioLut[16]  = 0.134320405034911;
	dutyRatioLut[17]  = 0.140045395290076;
	dutyRatioLut[18]  = 0.145676918699015;
	dutyRatioLut[19]  = 0.15122325951722;
	dutyRatioLut[20]  = 0.156691722461621;
	dutyRatioLut[21]  = 0.16209620864658;
	dutyRatioLut[22]  = 0.167424865448143;
	dutyRatioLut[23]  = 0.172693098842894;
	dutyRatioLut[24]  = 0.177912519088462;
	dutyRatioLut[25]  = 0.183070447328607;
	dutyRatioLut[26]  = 0.188180823379527;
	dutyRatioLut[27]  = 0.19325195176383;
	dutyRatioLut[28]  = 0.198274436613858;
	dutyRatioLut[29]  = 0.203264142427839;
	dutyRatioLut[30]  = 0.208210881753552;
	dutyRatioLut[31]  = 0.213129890553455;
	dutyRatioLut[32]  = 0.218011374203396;
	dutyRatioLut[33]  = 0.222869126110957;
	dutyRatioLut[34]  = 0.227694509584266;
	dutyRatioLut[35]  = 0.232499397366007;
	dutyRatioLut[36]  = 0.237276754451929;
	dutyRatioLut[37]  = 0.242036313360153;
	dutyRatioLut[38]  = 0.246772837448294;
	dutyRatioLut[39]  = 0.251493898630255;
	dutyRatioLut[40]  = 0.256198567914745;
	dutyRatioLut[41]  = 0.260884881400522;
	dutyRatioLut[42]  = 0.265560155390916;
	dutyRatioLut[43]  = 0.27022093086575;
	dutyRatioLut[44]  = 0.274872398366079;
	dutyRatioLut[45]  = 0.279514181795956;
	dutyRatioLut[46]  = 0.284145753782019;
	dutyRatioLut[47]  = 0.288771414410729;
	dutyRatioLut[48]  = 0.293390951819405;
	dutyRatioLut[49]  = 0.298004437920659;
	dutyRatioLut[50]  = 0.302614990341819;
	dutyRatioLut[51]  = 0.307222768594866;
	dutyRatioLut[52]  = 0.311828543737683;
	dutyRatioLut[53]  = 0.316434101769623;
	dutyRatioLut[54]  = 0.321040319572977;
	dutyRatioLut[55]  = 0.325648065397094;
	dutyRatioLut[56]  = 0.330258377557901;
	dutyRatioLut[57]  = 0.334873134725276;
	dutyRatioLut[58]  = 0.339492098667328;
	dutyRatioLut[59]  = 0.344116741448729;
	dutyRatioLut[60]  = 0.348749688967229;
	dutyRatioLut[61]  = 0.353389436373862;
	dutyRatioLut[62]  = 0.358037990783945;
	dutyRatioLut[63]  = 0.362698903331048;
	dutyRatioLut[64]  = 0.367369120103118;
	dutyRatioLut[65]  = 0.372053898846813;
	dutyRatioLut[66]  = 0.376750313855417;
	dutyRatioLut[67]  = 0.381460971667653;
	dutyRatioLut[68]  = 0.386190748502108;
	dutyRatioLut[69]  = 0.390934601616577;
	dutyRatioLut[70]  = 0.395699996161431;
	dutyRatioLut[71]  = 0.400482003240632;
	dutyRatioLut[72]  = 0.405288112156166;
	dutyRatioLut[73]  = 0.410113475571925;
	dutyRatioLut[74]  = 0.414965728095139;
	dutyRatioLut[75]  = 0.419839938311929;
	dutyRatioLut[76]  = 0.424744152032975;
	dutyRatioLut[77]  = 0.429673035499853;
	dutyRatioLut[78]  = 0.434635477381593;
	dutyRatioLut[79]  = 0.439625248714515;
	dutyRatioLut[80]  = 0.444652702421609;
	dutyRatioLut[81]  = 0.449710021745231;
	dutyRatioLut[82]  = 0.454809862486662;
	dutyRatioLut[83]  = 0.459947916029706;
	dutyRatioLut[84]  = 0.465122177175669;
	dutyRatioLut[85]  = 0.470345069724231;
	dutyRatioLut[86]  = 0.475612943160088;
	dutyRatioLut[87]  = 0.480923235668279;
	dutyRatioLut[88]  = 0.486290391137753;
	dutyRatioLut[89]  = 0.491710502902218;
	dutyRatioLut[90]  = 0.497186576077871;
	dutyRatioLut[91]  = 0.502714978452635;
	dutyRatioLut[92]  = 0.508313670548894;
	dutyRatioLut[93]  = 0.513978386661947;
	dutyRatioLut[94]  = 0.519712981176403;
	dutyRatioLut[95]  = 0.525521574016954;
	dutyRatioLut[96]  = 0.531408572298795;
	dutyRatioLut[97]  = 0.537378693999249;
	dutyRatioLut[98]  = 0.543436993867235;
	dutyRatioLut[99]  = 0.549588891813338;
	dutyRatioLut[100] = 0.555850798987246;
	dutyRatioLut[101] = 0.562210709003058;
	dutyRatioLut[102] = 0.568683779101318;
	dutyRatioLut[103] = 0.575277334313471;
	dutyRatioLut[104] = 0.582015539193605;
	dutyRatioLut[105] = 0.588880849581754;
	dutyRatioLut[106] = 0.595912111281843;
	dutyRatioLut[107] = 0.603106328668573;
	dutyRatioLut[108] = 0.610465703673844;
	dutyRatioLut[109] = 0.618033550365233;
	dutyRatioLut[110] = 0.625813084350466;
	dutyRatioLut[111] = 0.633824958537475;
	dutyRatioLut[112] = 0.642109531133249;
	dutyRatioLut[113] = 0.65066526642144;
	dutyRatioLut[114] = 0.659552553188346;
	dutyRatioLut[115] = 0.668784180698048;
	dutyRatioLut[116] = 0.678437776452109;
	dutyRatioLut[117] = 0.688555182308474;
	dutyRatioLut[118] = 0.699232422137456;
	dutyRatioLut[119] = 0.71054085405537;
	dutyRatioLut[120] = 0.722607432706813;
	dutyRatioLut[121] = 0.735591756494132;
	dutyRatioLut[122] = 0.749778719464673;
	dutyRatioLut[123] = 0.765462247180981;
	dutyRatioLut[124] = 0.783208914203245;
	dutyRatioLut[125] = 0.803957030968215;
	dutyRatioLut[126] = 0.829691366925824;
	dutyRatioLut[127] = 0.865848856727283;
	dutyRatioLut[128] = 1.0;
}

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
  while (1)
  {
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
