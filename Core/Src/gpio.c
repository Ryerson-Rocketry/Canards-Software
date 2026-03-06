/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GNSS_GPIO_Pin|SPI2_CS_2_Pin|RADIO_G0_Pin|Servo_HS_Sw_Pin
                          |Radio_EN_Pin|SPI1_CS_Pin|RADIO_RST_Pin|Radio_RST_Pin
                          |RX_LED_GP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Radio_D5_Pin|Radio_D4_Pin|SPI1_CS_3_Pin|GPIO_1_Pin
                          |GPIO_2_Pin|SDIO_NCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2__CS_Pin|GPIO_6_Pin|GPIO_5_Pin|GPIO_4_Pin
                          |GPIO_3_Pin|GPIO_LED_Pin|SPI1_CS_3B7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GNSS_GPIO_Pin SPI2_CS_2_Pin RADIO_G0_Pin Servo_HS_Sw_Pin
                           Radio_EN_Pin SPI1_CS_Pin RADIO_RST_Pin Radio_RST_Pin
                           RX_LED_GP_Pin */
  GPIO_InitStruct.Pin = GNSS_GPIO_Pin|SPI2_CS_2_Pin|RADIO_G0_Pin|Servo_HS_Sw_Pin
                          |Radio_EN_Pin|SPI1_CS_Pin|RADIO_RST_Pin|Radio_RST_Pin
                          |RX_LED_GP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Radio_D5_Pin Radio_D4_Pin SPI1_CS_3_Pin GPIO_1_Pin
                           GPIO_2_Pin SDIO_NCD_Pin */
  GPIO_InitStruct.Pin = Radio_D5_Pin|Radio_D4_Pin|SPI1_CS_3_Pin|GPIO_1_Pin
                          |GPIO_2_Pin|SDIO_NCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2__CS_Pin GPIO_6_Pin GPIO_5_Pin GPIO_4_Pin
                           GPIO_3_Pin GPIO_LED_Pin SPI1_CS_3B7_Pin */
  GPIO_InitStruct.Pin = SPI2__CS_Pin|GPIO_6_Pin|GPIO_5_Pin|GPIO_4_Pin
                          |GPIO_3_Pin|GPIO_LED_Pin|SPI1_CS_3B7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
