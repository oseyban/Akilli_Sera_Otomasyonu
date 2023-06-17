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
#include "dht22.h"
#include "stdio.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t data=0;
int data1=0;
char yazi3[16]="";

uint32_t data2=0;
int data3=0;
char yazi4[16]="";
int sayac=0;
char yazi5[16]="";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float nem,sicaklik;
char yazi1[16]="";
char yazi2[16]="";
int sicaklik1=0,nem1=0;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  	  dht22_init();
  	  lcd_init(_LCD_4BIT, _LCD_FONT_5x10, _LCD_2LINE);
  	  HAL_Delay(500);

  	  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 2);
  	  //HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 2);

  	  HAL_ADC_Start(&hadc1);
  	  //HAL_ADC_Start(&hadc2);

  	HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  DHT22_GetTemp_Humidity(&sicaklik,&nem);
	  sicaklik1 = (int)sicaklik;
	  nem1 = (int)nem;
	  sprintf(yazi1, "Sicaklik=%d", sicaklik1);
	  sprintf(yazi2, "Nem=%d", nem1);
	  lcd_print(1,1,yazi1);
	  lcd_print(2,1,yazi2);
	  if((sicaklik>28)||(nem>80)){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  lcd_clear();
		  lcd_print(1,1,"Yuksek Nem&Sicak");
		  lcd_print(2,1,"Fan Devrede");
		  }

	  else if(sicaklik<26){
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  		  lcd_clear();
	  		  lcd_print(1,1,"Dusuk Sicaklik");
	  		  lcd_print(2,1,"Isitma Devrede");
	  		  }
	  else if(nem<78){
	  	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	  	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	  		  lcd_clear();
	  	  		  lcd_print(1,1,"Dusuk Nem");
	  	  		  lcd_print(2,1,"Nemlendirme Devrede");
	  	  		  }
	else
		  {
		  lcd_clear();
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  lcd_print(1,1,yazi1);
		  lcd_print(2,1,yazi2);
		  }
	  lcd_delay();


	  	  /* ADC1 sonuçlarını işleme */
	  	  //uint32_t adc1_ch0 = adc1_buffer[0]; // ADC1 kanal 0 sonucunu oku
	      //uint32_t adc1_ch1 = adc1_buffer[1]; // ADC1 kanal 1 sonucunu oku

	      /* ADC2 sonuçlarını işleme */
	      //uint32_t adc2_ch0 = adc2_buffer[0]; // ADC2 kanal 0 sonucunu oku
	      //uint32_t adc2_ch1 = adc2_buffer[1];

	  // toprak Nem
	  data=HAL_ADC_GetValue(&hadc1);
	  data1=(int)data;

	  	  if (data<600){
	  		  sprintf(yazi3, "T.Nem Degeri=%d", data1);
	  		  lcd_print(1,1,"Toprak Nem Dusuk");
	  		  lcd_print(2,1,yazi3);
	  		  HAL_Delay(100);
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	  	  }
	  	  else {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);}

	 /* // toprak PH
	  data2=HAL_ADC_GetValue(&hadc2);
	  data3=(int)data2;

	  	  if (data3<600){
	  	  sprintf(yazi4, "T.PH Degeri=%d", data3);
	  	  lcd_print(1,1,"Toprak PH Dusuk");
	  	  lcd_print(2,1,yazi4);
	  	  lcd_delay();
	  	  lcd_print(2,1,"Kükürtleme Yapınız");

	  	  HAL_Delay(100);
	  	  }
	  	  else if (data3>800){sprintf(yazi4, "T.PH Degeri=%d", data3);
	  	  lcd_print(1,1,"Toprak PH Yuksek");
	  	  lcd_print(2,1,yazi4);
	  	  lcd_delay();
	  	  lcd_print(2,1,"Asitli Gubre Kull");

	  	  HAL_Delay(100);}
	  	*/



	  HAL_Delay(500);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_RS_Pin|LCD_POT_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|kapak_motor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DHT22_Pin|motor1_Pin|motor2_Pin|motor3_Pin
                          |LED_Pin|Panel_Motor_Pin|motor4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin LCD_POT_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin kapak_motor_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|LCD_POT_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|kapak_motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT22_Pin motor1_Pin motor2_Pin motor3_Pin
                           motor4_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin|motor1_Pin|motor2_Pin|motor3_Pin
                          |motor4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin Panel_Motor_Pin */
  GPIO_InitStruct.Pin = LED_Pin|Panel_Motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	sayac++;
	if ((sayac>=2)&&(sayac<=5)){
		lcd_print(1,1,"Gece Aydinlatmasi");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	}


/*
	static uint8_t is_daytime = 0; // Gündüz / gece durumu için bayrak

	  uint32_t current_hour = HAL_GetTick() / 3600000; // Sistem zamanını saat cinsinden al

	  if (current_hour >= 7 && current_hour < 19) // Gündüz saatleri
	  {
	    if (!is_daytime)
	    {
	      is_daytime = 1; // Gündüz olduğunu işaretlemek için bayrağı ayarla
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);     // Işık pinini kapalı duruma getir
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);   // Perde pinini açık duruma getir
	      lcd_print(1, 1, "Gündüz Oldu");
	      HAL_Delay(10000); // 10 saniye bekle
	    }
	  }
	  else // Gece saatleri
	  {
	    if (is_daytime)
	    {
	      is_daytime = 0; // Gece olduğunu işaretlemek için bayrağı sıfırla
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);   // Işık pinini açık duruma getir
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);     // Perde pinini kapalı duruma getir
	      lcd_print(1, 1, "Gece Oldu");
	      HAL_Delay(10000); // 10 saniye bekle
	    }
	  }
	}
*/



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
