/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "functions.h"
#include "stm32g0xx_hal_flash.h"
#include "ssd1306.h"
#include "ssd1306_defines.h"
#include "eeG0.h"
#include "picture_test.c"
#include "fonts.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;

/* USER CODE BEGIN PV */

uint32_t* VREFINT_CAL = (uint32_t*)0x1FFFF7BA;
uint8_t edit_string = 0;//экран для редактирования
uint8_t screen = 0;//номер экрана
uint8_t edit = 0;
uint8_t last_edit; // сохранение значения для вычисления спада события
uint8_t first_run = 1;
uint32_t hexPage;
uint32_t Time_save[5];
uint8_t Mode=0;
uint16_t adc[30]; // массив для хранения опросов АЦП
enum result {
	QCERROR,
	QC2NOT,
	QC2_20VNOT,
	QC3_20VNOT,
	QC3NOT,
	QC3OK,
};
// результаты АЦП
typedef struct
{
	uint32_t Uvdda; /* напряжение микроконтроллера */
	uint32_t Ioutput; /* ток на выходе */
	uint32_t Uoutput; /* напряжение на выходе */
	uint32_t Temperature; /* температура ядра микроконтроллера */
} ADC_measurement_DataType;

ADC_measurement_DataType Measurement;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void QC3_5V(void);
void QC3_9V(void);
void QC3_12V(void);
void QC3_20V(void);
void QC3_Reg(void);
void QC3_Inc(void);
void QC3_Dec(void);
void ADC_measurement(ADC_measurement_DataType *measurement, uint16_t *ADC_array);
uint8_t Test(void);
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init(STM32_I2C_PORT, SSD1306_ADDRESS);
  //--------------------- чтение из ФЛЕШ сохраненных значений , если не первый пуск
 /* hexPage = getHexAddressPage(DATA_PAGE);            //Get our hex page

  	  first_run = retrieveDataFromAddress(hexPage + (DATA_SPACE * 0));
  	  if (first_run)
  	  {
  		first_run=0;
  		writeThreeData(hexPage, first_run, Set_Illumination, Set_range);
  	  }
  	  else
  		  {
  		  Set_Illumination = retrieveDataFromAddress(hexPage + (DATA_SPACE * 1));
	  	  Set_range = retrieveDataFromAddress(hexPage + (DATA_SPACE * 2));
  		  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	HAL_ADCEx_Calibration_Start(&hadc1);
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 30);

  	//QC3_Reg();
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	uint8_t data =0;
	switch ( Mode )
	  {
	      // declarations
	      // . . .
	      case 0:

	    	  ssd1306_Fill(White);
	    	  ssd1306_SetCursor(32,4);
	    	  ssd1306_WriteString("TEST", font16x24, Black);
	    	  //ssd1306_picture(43,6,picture_Test,44,21,Black);
	    	  ssd1306_UpdateScreen();
	    	  HAL_Delay(1000);


	    	  uint8_t data = Test();
	    	  ssd1306_Fill(Black);

	    	  	  //HAL_Delay(50);
	    	  	  if (data == QCERROR) {
	    	  		  //ssd1306_picture(35,6,picture_Error,57,21,White);
	    	  		ssd1306_SetCursor(24,4);
	    	  		ssd1306_WriteString("ERROR", font16x24, White);
		    	  	ssd1306_UpdateScreen();
	    	  		while (1){};
	    	  	  }
	    	  	  if (data == QC2NOT) {
	    	    	  ssd1306_Fill(Black);
	    	    	  //ssd1306_picture(25,6,picture_QC2,46,21,White);
	    	    	  //ssd1306_picture(73,0,picture_NOTOK,32,32,White);
	    	    	  ssd1306_SetCursor(24,4);
	    	    	  ssd1306_WriteString("NO QC", font16x24, White);
	    	    	  ssd1306_UpdateScreen();
	    	  		while (1){};
	    	  	  }
	    	  	  if (data == QC2_20VNOT) {
	    	  		  //ssd1306_picture(2,5,picture_Only,53,24,White);
	    	  		  //ssd1306_picture(58,6,picture_QC2,46,21,White);
	    	  		  //ssd1306_picture(107,6,picture_A,17,21,White);
	    	  		  ssd1306_SetCursor(0,4);
	    	  		  ssd1306_WriteString("OnlyQC2A", font16x24, White);
		    	  	  ssd1306_UpdateScreen();
	    	  		while (1){};
	    	  	  }
	    	  	  if (data == QC3NOT) {
	    	  		  //ssd1306_picture(2,5,picture_Only,53,24,White);
	    	  		  //ssd1306_picture(58,6,picture_QC2,46,21,White);
	    	  		  //ssd1306_picture(107,6,picture_B,17,21,White);
	    	  		  ssd1306_SetCursor(16,4);
	    	  		  ssd1306_WriteString("NO QC3", font16x24, White);
		    	  	  ssd1306_UpdateScreen();
	    	  		while (1){};
	    	  	  }
	    	  	  if (data == QC3_20VNOT) {
	    	  		  //ssd1306_picture(2,5,picture_Only,53,24,White);
	    	  		  //ssd1306_picture(58,6,picture_QC3,46,21,White);
	    	  		  //ssd1306_picture(107,6,picture_A,17,21,White);
	    	  		  ssd1306_SetCursor(24,4);
	    	  		  ssd1306_WriteString("OnlyQC3A", font16x24, White);
		    	  	  ssd1306_UpdateScreen();
	    	  		while (1){};
	    	  	  }
	    	   	  if (data == QC3OK) {
	    	    	  ssd1306_Fill(Black);
	    	    	  //ssd1306_picture(14,5,picture_QC3,46,21,White);
	    	    	  //ssd1306_picture(62,6,picture_B,17,21,White);
	    	    	  //ssd1306_picture(83,0,picture_OK,32,32,White);
	    	  		  ssd1306_SetCursor(16,4);
	    	  		  ssd1306_WriteString("QC3 OK", font16x24, White);
	    	    	  ssd1306_UpdateScreen();
	    	    	  HAL_Delay(1000);
	    	  		  Mode=1;
	    	  		    	  	  }
	    	  	  break;
	      case 1:

	          // statements executed if the expression equals the
	          // value of this constant_expression
	    		if (Time_save[1] + 200 < HAL_GetTick()) // опрос аналоговых входов 200мс
	    				{
	    					ADC_measurement(&Measurement, adc);
	    					Time_save[1] = HAL_GetTick();
	    				}

	    		ssd1306_Fill(Black);
	    		ssd1306_SetCursor(0,4);
	    		const char *string1 =  {"U="};
	    		const char voltage []= {'V'};
	    		ssd1306_WriteString(string1, font16x24, White);
	    		ssd1306_WriteNum(Measurement.Uoutput, 2, voltage, White, font16x24);
/*
	    		ssd1306_SetCursorPos(0, 1, font16x24);
	    		const char *string2 =  {"I= "};
	    		const char current []= {' ','A',' '};
	    		ssd1306_WriteString(string2, font16x24, White);
	    		ssd1306_WriteNum(Measurement.Ioutput, 2, current, White, font16x24);

	    		ssd1306_SetCursorPos(0, 2, font16x24);
	    		const char *string3 =  {"Vdd= "};
	    		const char voltage_mV []= {' ','m','V'};
	    		ssd1306_WriteString(string3, font16x24, White);
	    		ssd1306_WriteNum(Measurement.Uvdda, 0, voltage_mV, White, font16x24);

*/
	    		ssd1306_UpdateScreen();
	    		HAL_Delay(50);
	          break;

	      default:
	          // statements executed if expression does not equal
	          // any case constant_expression
	  }




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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00701F6F;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Dp2_Pin|Dn2_Pin|Dn2_06_Pin|Dp2_06_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Dp2_Pin Dn2_Pin Dn2_06_Pin Dp2_06_Pin */
  GPIO_InitStruct.Pin = Dp2_Pin|Dn2_Pin|Dn2_06_Pin|Dp2_06_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Dp_0V  (void) {HAL_GPIO_WritePin(GPIOA, Dp2_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, Dp2_06_Pin, GPIO_PIN_SET);}
void Dn_0V  (void) {HAL_GPIO_WritePin(GPIOA, Dn2_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, Dn2_06_Pin, GPIO_PIN_SET);}
void Dp_0V6 (void) {HAL_GPIO_WritePin(GPIOA, Dp2_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, Dp2_06_Pin, GPIO_PIN_RESET);}
void Dn_0V6 (void) {HAL_GPIO_WritePin(GPIOA, Dn2_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, Dn2_06_Pin, GPIO_PIN_RESET);}
void Dp_3V3 (void) {HAL_GPIO_WritePin(GPIOA, Dp2_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, Dp2_06_Pin, GPIO_PIN_SET);}
void Dn_3V3 (void) {HAL_GPIO_WritePin(GPIOA, Dn2_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, Dn2_06_Pin, GPIO_PIN_SET);}

void QC3_enter(void)
{
	/* DP: 0.6V; DN: 0.6V - preset */
	Dp_0V6();
	Dn_0V6();
	HAL_Delay(1400); /* min 1.25s */

  	/* DP: 0.6V; DN: 0V */
	Dn_0V();
 	HAL_Delay(1); /* min 1ms */
//	Dn_0V6();
}


void QC3_5V(void)
{
    /* DP: 0.6V; DN: 0.6V - preset */
    /* DP: 0.6V; DN: 0V */
    Dp_0V6();
    Dn_0V();



}
void QC3_9V(void)
{

    /* DP: 3.3V; DN: 0.6V for 9V */
    Dp_3V3();
    Dn_0V6();
}

void QC3_12V(void)
{
    /* DP: 0.6V; DN: 0.6V for 12V */
    Dp_0V6();
    Dn_0V6();
}

void QC3_20V(void)
{

    /* DP: 3.3V; DN: 3.3V for 20V */
    Dp_3V3();
    Dn_3V3();
}

void QC3_Reg(void)
{

    /* DP: 0.6V; DN: 3.3V for Reg */
    Dp_0V6();
    Dn_3V3();


}

void QC3_Inc(void){

	 /*     ******     */
	 /*     *	 *     */
	 /*     *	 *     */
	 /*******    *******/

	 /* DP: 3.3V */
	 Dp_3V3();
	 HAL_Delay(1);
	 /* DP: 0.6V */
	 Dp_0V6();
	 HAL_Delay(2);
}

void QC3_Dec(void){

	 /*******    *******/
	 /*     *	 *     */
	 /*     *	 *     */
	 /*     ******     */

	 /* DN: 0.6V */
	 Dn_0V6();
	 HAL_Delay(1);
	 /* DN: 3.3V */
	 Dn_3V3();
	 HAL_Delay(2);

}





uint8_t Test(void)
{
 QC3_enter();
 QC3_5V();
 HAL_Delay(100);
 ADC_measurement(&Measurement, adc);
//	  ssd1306_SetCursorPos(0, 2, Font_7x9);
//	  ssd1306_WriteNum(Measurement.Uoutput, 2, " 5V", White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
 if (Measurement.Uoutput < 480 || Measurement.Uoutput > 520) {QC3_5V(); return (QCERROR);}
 QC3_9V();
 HAL_Delay(100);
 ADC_measurement(&Measurement, adc);
//	  ssd1306_SetCursorPos(0, 2, Font_7x9);
//	  ssd1306_WriteNum(Measurement.Uoutput, 2, " 9V", White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
 if (Measurement.Uoutput < 880 || Measurement.Uoutput > 920) {QC3_5V(); return (QC2NOT);}
 QC3_20V();
 HAL_Delay(100);
 ADC_measurement(&Measurement, adc);
//	  ssd1306_SetCursorPos(0, 2, Font_7x9);
//	  ssd1306_WriteNum(Measurement.Uoutput, 2, " 20", White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
 if (Measurement.Uoutput < 1900 || Measurement.Uoutput > 2300) {QC3_5V(); return (QC2_20VNOT);}
 QC3_5V();
 HAL_Delay(100);
 ADC_measurement(&Measurement, adc);
// ssd1306_SetCursorPos(0, 2, Font_7x9);
// ssd1306_WriteNum(Measurement.Uoutput, 2, " 5V", White);
// ssd1306_UpdateScreen();
// HAL_Delay(1000);
 QC3_Reg();
 HAL_Delay(100);
 ADC_measurement(&Measurement, adc);
// ssd1306_SetCursorPos(0, 2, Font_7x9);
// ssd1306_WriteNum(Measurement.Uoutput, 2, " Rg", White);
 //ssd1306_UpdateScreen();
// HAL_Delay(1000);

 for (uint8_t i = 0; i < 10; i++)
 {
  QC3_Inc();
 }
 HAL_Delay(1000);
 ADC_measurement(&Measurement, adc);
//	  ssd1306_SetCursorPos(0, 2, Font_7x9);
//	  ssd1306_WriteNum(Measurement.Uoutput, 2, " 7V", White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
 if (Measurement.Uoutput < 680 || Measurement.Uoutput > 720) return (QC3NOT);
 for (uint8_t i = 0; i < 55; i++)
 {
  QC3_Inc();
 }
 HAL_Delay(1000);
 ADC_measurement(&Measurement, adc);
//	  ssd1306_SetCursorPos(0, 2, Font_7x9);
//	  ssd1306_WriteNum(Measurement.Uoutput, 2, " QC18V ", White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
 if (Measurement.Uoutput < 1770 || Measurement.Uoutput > 1830) return (QC3_20VNOT);
 QC3_5V();
 return (QC3OK);

}

void ADC_measurement(ADC_measurement_DataType *measurement, uint16_t *ADC_array)
{

//uint32_t Uoutput_32=0; // измеренное напряжение
//uint32_t Ioutput_32=0; // измеренный ток
//uint32_t Uvdda_32=0; //  измерение напряжения питания АЦП
measurement->Uvdda =0;
measurement->Uoutput =0;
measurement->Ioutput =0;

for (uint8_t i = 0; i<30; i+=3) // берем сумму каждых четных адресов
	{
	measurement->Uoutput = measurement->Uoutput + ADC_array[i];
	}
for (uint8_t i = 1; i<30; i+=3) // берем сумму каждых четных адресов
  	{
	measurement->Ioutput = measurement->Ioutput + ADC_array[i];
  	}
for (uint8_t i = 2; i<30; i+=3) // берем сумму каждых четных адресов
  	{
	measurement->Uvdda = measurement->Uvdda + ADC_array[i];
  	}
measurement->Uvdda = (4095*12300/measurement->Uvdda); //напряжение в мВ на пине VDDA
//measurement->Uvdda = median(measurement->Uvdda);
// на входе делитель напряжения в 6,68 раз
measurement->Uoutput = measurement->Uoutput*measurement->Uvdda/40950; //получаем значение в мВ
measurement->Uoutput = measurement->Uoutput*668/1000;
// тока 264мВ на 1А, датчик 5А, максимальный размах 1 320мВ, 0А = Uvdd/2
measurement->Ioutput = measurement->Ioutput*measurement->Uvdda/40950-measurement->Uvdda/2; // получаем значение в мВ откорректирование к реальному напряжению на входе VDDA
measurement->Ioutput = measurement->Ioutput*100/264;
//measurement->Ioutput = median(measurement->Ioutput);
measurement->Ioutput = limit(measurement->Ioutput,0,500);//ограничиваем показания
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
