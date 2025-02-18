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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "string.h"
#include <stdio.h>
#include "math.h"
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
MPU6050_t MPU6050;
const float calibPiezo[5] = {0.75,0.5,1.2,1.0,1.0};
const int xHitCoords[5] = {-1,1,0,-1,1};
const int yHitCoords[5] = {1,1,0,-1,-1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADC_Select_CH(int ch){
	ADC_ChannelConfTypeDef sConfig = {0};
	uint32_t channel;
	switch(ch){
	case 0:
		channel = ADC_CHANNEL_0;
		break;
	case 1:
		channel = ADC_CHANNEL_1;
		break;
	case 2:
		channel = ADC_CHANNEL_2;
		break;
	case 3:
		channel = ADC_CHANNEL_3;
		break;
	case 4:
		channel = ADC_CHANNEL_4;
		break;
	default:
		return;
		break;
	}
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

uint16_t ADC_Read(int ch){
	uint16_t AD_VAL;
	ADC_Select_CH(ch);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	AD_VAL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return AD_VAL*calibPiezo[ch];
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  HAL_Delay(50);
  /* USER CODE BEGIN 2 */
  while (MPU6050_Init(&hi2c1) == 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
  /* USER CODE END 2 */
  uint16_t piezoWindowBuf [5][250] = {0};
  uint16_t maxs [5] = {0};
  uint16_t absMax;
  float xHitLoc,yHitLoc;
  float weights[5] = {0.0};
  int triggerdIterator =0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint16_t AD_RES = 0;
	  char adcval[128];
    //Constant reading of the 5 piezo discs and a rolling window in the first half of the buffer
    //The most recent reading is in the middle of the buffer
	  if(triggerdIterator == 0){
		  for(int i=0;i<5;i++){
			  for(int j = 0; j<124; j++){
				  piezoWindowBuf[i][j] = piezoWindowBuf[i][j+1];
			  }
			  piezoWindowBuf[i][124] = ADC_Read(i);
        //Detect a ball hit if any of the sensors pass the threshold
			  if(piezoWindowBuf[i][124] > 1500){
				  HAL_Delay(1);  //Wait for a millisecond and check again to ignore spikes
				  if(ADC_Read(i)> 1500){ 
					  triggerdIterator =125;
				  }
			  }
		  }
      //If a hit is detected, fill the rest of the window
	  }else if(triggerdIterator < 250){
		  for(int i=0;i<5;i++){
			  piezoWindowBuf[i][triggerdIterator] = ADC_Read(i);
		  }
		  triggerdIterator++;
	  }
    //When the rest of the buffer is filled, calculate and send the data
    else{
		  triggerdIterator =0;
//		  for(int i =0;i<250;i++){
//			  for(int j=0;j<5;j++){
//	  				sprintf(adcval,"%d",piezoWindowBuf[j][i]);
//	  				HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
//	  				HAL_UART_Transmit(&huart1, (uint8_t *) "\t" , 1, HAL_MAX_DELAY);
//			  }
//			  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n" , 2, HAL_MAX_DELAY);
//		  			  }

//Determines the maximum value in each buffer
		  for(int i=0;i<5;i++){
			  for(int j=0;j<250;j++){
				  if (piezoWindowBuf[i][j]>maxs[i]){
					  maxs[i] = piezoWindowBuf[i][j];
				  }
			  }

		  }
// Determine the absolute maximum of the 5 sensors
		  absMax = 0;
		  for(int k=0;k<5;k++){
			  if(maxs[k]>absMax){
				  absMax = maxs[k];
			  }
		  }
//Normalize and calculate the location
		  for(int k=0;k<5;k++){
				  weights[k] = (float) maxs[k]/absMax;
				  xHitLoc += weights[k]*xHitCoords[k];
				  yHitLoc += weights[k]*yHitCoords[k];
		  }
		  	HAL_Delay(5);
		  	MPU6050_Read_All(&hi2c1, &MPU6050);

      //send the data
			sprintf(adcval,"%f",xHitLoc);
			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t *) "," , 1, HAL_MAX_DELAY);
			sprintf(adcval,"%f",yHitLoc);
			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t *) "," , 1, HAL_MAX_DELAY);
			sprintf(adcval,"%f",MPU6050.KalmanAngleX);
			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t *) "," , 1, HAL_MAX_DELAY);
			sprintf(adcval,"%f",MPU6050.KalmanAngleY);
			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t *) "," , 1, HAL_MAX_DELAY);
			float force = fabs(sqrt(pow(MPU6050.Ax,2)+pow(MPU6050.Ay,2)+pow(MPU6050.Az,2))-1)*9.8*0.0027;
			sprintf(adcval,"%f",force);
			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
      //Training example test
			if(force>0.015){
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
				  HAL_Delay(50);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			}
      //Reset all values
		  for(int i =0;i<5;i++){
//		  sprintf(adcval,"%f",weights[i]);
//			HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart1, (uint8_t *) "\t" , 1, HAL_MAX_DELAY);
			maxs[i] = 0;
			weights[i] = 0;
			xHitLoc = 0;
			yHitLoc = 0;
	  }
		  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n" , 2, HAL_MAX_DELAY);


	  }
//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n" , 2, HAL_MAX_DELAY);
//      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//      sprintf(adcval,"%d",AD_RES);
//      HAL_UART_Transmit(&huart1, (uint8_t *) adcval , strlen(adcval), HAL_MAX_DELAY);
//      HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n" , 2, HAL_MAX_DELAY);
//	  char angleStr[128];
//	  sprintf(angleStr,"%f",MPU6050.Ax*9.80665);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) angleStr , strlen(angleStr), HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\t" , 1, HAL_MAX_DELAY);
//	  sprintf(angleStr,"%f",MPU6050.Ay*9.80665);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) angleStr , strlen(angleStr), HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\t" , 1, HAL_MAX_DELAY);
//	  sprintf(angleStr,"%f",MPU6050.Az*9.80665);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) angleStr , strlen(angleStr), HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n" , 2, HAL_MAX_DELAY);
//	  if(MPU6050.KalmanAngleX > 30){
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
//	  }else{
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//	  }
//	  HAL_Delay (20);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
