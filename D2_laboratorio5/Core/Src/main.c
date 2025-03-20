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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t estado=0;
uint16_t start=0;
uint16_t sumar1=0;
uint16_t sumar2=0;
uint16_t contador_j1=0;
uint16_t contador_j2=0;
uint16_t disp=5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void display(uint16_t numero);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display(uint16_t numero){
	switch(numero){
	case 1:
		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_RESET);
		break;
	case 0:
		HAL_GPIO_WritePin(dispa_GPIO_Port, dispa_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispb_GPIO_Port, dispb_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispc_GPIO_Port, dispc_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispd_GPIO_Port, dispd_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispe_GPIO_Port, dispe_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dispf_GPIO_Port, dispf_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(dispg_GPIO_Port, dispg_Pin, GPIO_PIN_RESET);
		break;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(estado==1){
		  estado=0;
		  for(int i=0;i<6;i++){
			  display(disp);
			  HAL_Delay(1000);
			  disp--;
		  }
		  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_RESET);
		  start=1;
	  }
	  if (start==1){
		  if (sumar1==1){
			  contador_j1++;
			  switch(contador_j1){
			  case 1:
				  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 2:
				  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 3:
				  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 4:
				  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_SET);
				  display(1);
				  start=0;
				  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_RESET);
				  break;
			  }
			  sumar1=0;
		  }
		  if(sumar2==1){
			  contador_j2++;
			  switch(contador_j2){
			  case 1:
				  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 2:
				  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 3:
				  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_RESET);
				  break;
			  case 4:
				  HAL_GPIO_WritePin(LEDj2_1_GPIO_Port, LEDj2_1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_2_GPIO_Port, LEDj2_2_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_3_GPIO_Port, LEDj2_3_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(LEDj2_4_GPIO_Port, LEDj2_4_Pin, GPIO_PIN_SET);
				  display(2);
				  start=0;
				  HAL_GPIO_WritePin(LEDj1_1_GPIO_Port, LEDj1_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_2_GPIO_Port, LEDj1_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_3_GPIO_Port, LEDj1_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LEDj1_4_GPIO_Port, LEDj1_4_Pin, GPIO_PIN_RESET);
				  break;
			  }
			  sumar2=0;
		  }
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|dispc_Pin|dispd_Pin|dispe_Pin
                          |dispf_Pin|dispg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEDj2_1_Pin|LEDj2_2_Pin|LEDj2_3_Pin|LEDj2_4_Pin
                          |dispa_Pin|dispb_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDj1_1_Pin|LEDj1_2_Pin|LEDj1_3_Pin|LEDj1_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : start_btn_Pin j1_btn_Pin j2_btn_Pin */
  GPIO_InitStruct.Pin = start_btn_Pin|j1_btn_Pin|j2_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin dispc_Pin dispd_Pin dispe_Pin
                           dispf_Pin dispg_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|dispc_Pin|dispd_Pin|dispe_Pin
                          |dispf_Pin|dispg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDj2_1_Pin LEDj2_2_Pin LEDj2_3_Pin LEDj2_4_Pin
                           dispa_Pin dispb_Pin */
  GPIO_InitStruct.Pin = LEDj2_1_Pin|LEDj2_2_Pin|LEDj2_3_Pin|LEDj2_4_Pin
                          |dispa_Pin|dispb_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDj1_1_Pin LEDj1_2_Pin LEDj1_3_Pin LEDj1_4_Pin */
  GPIO_InitStruct.Pin = LEDj1_1_Pin|LEDj1_2_Pin|LEDj1_3_Pin|LEDj1_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==start_btn_Pin){
		estado=1;
		contador_j1=0;
		contador_j2=0;
		sumar1=0;
		sumar2=0;
		disp=5;
	}
	else if(GPIO_Pin==j1_btn_Pin){
		sumar1=1;
	}
	else if(GPIO_Pin==j2_btn_Pin){
		sumar2=1;
	}
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
