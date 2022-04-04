/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void collect_sample_LL(uint8_t *rx_buffer);
void collect_sample_DRA(uint8_t *rx_buffer);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t rx_buffer[2];
  float temperature;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//	HAL_Delay(10);
	//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	//	HAL_SPI_Receive(&hspi2, rx_buffer, 2, 1);
	//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	wait_for_10_ms();

	collect_sample_LL(rx_buffer);
	// collect_sample_DRA(rx_buffer);

	int16_t tmp = (rx_buffer[0] << 8) | rx_buffer[1];

	if ((tmp & (1 << 13)) != 0) {
		tmp -= 16384;
	}
	temperature = (float)tmp;
	temperature /= 32.;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
}

/* USER CODE BEGIN 4 */
void collect_sample_DRA(uint8_t *rx_buffer) {
	WRITE_REG(GPIOB->BSRR, (LL_GPIO_PIN_0 << 16)); // \CS low
	wait_for_10_us();
	SET_BIT(SPI2->CR1, SPI_CR1_SPE); // SPI enable

	SPI2->DR = 0x00; // dummy write
	while ( !READ_BIT(SPI2->SR, SPI_SR_TXE) ); // wait until TXE is set
	SPI2->DR = 0x00; // dummy write
	while ( !READ_BIT(SPI2->SR, SPI_SR_RXNE) ); // wait until RXNE is set
	rx_buffer[0] = SPI2->DR; // read DR (clears RXNE)

	while ( !READ_BIT(SPI2->SR, SPI_SR_RXNE) ); // wait until RXNE is set
	rx_buffer[1] = SPI2->DR; // read DR (clears RXNE)

	while ( !READ_BIT(SPI2->SR, SPI_SR_TXE) ); // wait until TXE is set
	while ( READ_BIT(SPI2->SR, SPI_SR_BSY) ); // wait until BSY is reset

	CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE); // SPI disable
	wait_for_10_us();
	WRITE_REG(GPIOB->BSRR, (LL_GPIO_PIN_0)); // \CS high
}

void collect_sample_LL(uint8_t *rx_buffer) {
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0); // \CS low
	wait_for_10_us();
	LL_SPI_Enable(SPI2);

	LL_SPI_TransmitData8(SPI2, 0x00); // dummy write
	while ( !LL_SPI_IsActiveFlag_TXE(SPI2) ); // wait until TXE is set
	LL_SPI_TransmitData8(SPI2, 0x00); // dummy write
	while ( !LL_SPI_IsActiveFlag_RXNE(SPI2) ); // wait until RXNE is set
	rx_buffer[0] = LL_SPI_ReceiveData8(SPI2); // read DR (clears RXNE)

	while ( !LL_SPI_IsActiveFlag_RXNE(SPI2) ); // wait until RXNE is set
	rx_buffer[1] = LL_SPI_ReceiveData8(SPI2); // read DR (clears RXNE)

	while ( !LL_SPI_IsActiveFlag_TXE(SPI2) ); // wait until TXE is set
	while ( LL_SPI_IsActiveFlag_BSY(SPI2) ); // wait until BSY is reset

	LL_SPI_Disable(SPI2); // SPI disable
	wait_for_10_us();
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0); // \CS high
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
