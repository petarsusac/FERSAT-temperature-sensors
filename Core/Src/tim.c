/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 16;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/* USER CODE BEGIN 1 */
void wait_for_10_us() {
	// 16 MHz APB1 clock before timer prescaler
	LL_TIM_SetPrescaler(TIM2, 16); // 1 MHz after prescaler
	LL_TIM_SetAutoReload(TIM2, 10); // set auto-reload value
	LL_TIM_GenerateEvent_UPDATE(TIM2); // generate update event
	LL_TIM_EnableCounter(TIM2); // enable counter
	while ( !LL_TIM_IsActiveFlag_UPDATE(TIM2) ); // wait for update flag
	LL_TIM_ClearFlag_UPDATE(TIM2); // clear update flag
}

void wait_for_10_ms() {
	// 16 MHz APB1 clock before timer prescaler
	LL_TIM_SetPrescaler(TIM2, 16); // 1 MHz after prescaler
	LL_TIM_SetAutoReload(TIM2, 10000); // set auto-reload value
	LL_TIM_GenerateEvent_UPDATE(TIM2); // generate update event
	LL_TIM_EnableCounter(TIM2); // enable counter
	while ( !LL_TIM_IsActiveFlag_UPDATE(TIM2) ); // wait for update flag
	LL_TIM_ClearFlag_UPDATE(TIM2); // clear update flag
}
/* USER CODE END 1 */
