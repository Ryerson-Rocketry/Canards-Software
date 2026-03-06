/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GNSS_GPIO_Pin GPIO_PIN_13
#define GNSS_GPIO_GPIO_Port GPIOC
#define SPI2_CS_2_Pin GPIO_PIN_14
#define SPI2_CS_2_GPIO_Port GPIOC
#define RADIO_G0_Pin GPIO_PIN_15
#define RADIO_G0_GPIO_Port GPIOC
#define Servo_HS_Sw_Pin GPIO_PIN_0
#define Servo_HS_Sw_GPIO_Port GPIOC
#define Radio_EN_Pin GPIO_PIN_3
#define Radio_EN_GPIO_Port GPIOC
#define Radio_D5_Pin GPIO_PIN_1
#define Radio_D5_GPIO_Port GPIOA
#define Radio_D4_Pin GPIO_PIN_2
#define Radio_D4_GPIO_Port GPIOA
#define SPI1_CS_3_Pin GPIO_PIN_4
#define SPI1_CS_3_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define RADIO_RST_Pin GPIO_PIN_5
#define RADIO_RST_GPIO_Port GPIOC
#define SPI2__CS_Pin GPIO_PIN_0
#define SPI2__CS_GPIO_Port GPIOB
#define GPIO_6_Pin GPIO_PIN_1
#define GPIO_6_GPIO_Port GPIOB
#define GPIO_5_Pin GPIO_PIN_12
#define GPIO_5_GPIO_Port GPIOB
#define GPIO_4_Pin GPIO_PIN_13
#define GPIO_4_GPIO_Port GPIOB
#define GPIO_3_Pin GPIO_PIN_14
#define GPIO_3_GPIO_Port GPIOB
#define GPIO_LED_Pin GPIO_PIN_15
#define GPIO_LED_GPIO_Port GPIOB
#define Radio_RST_Pin GPIO_PIN_6
#define Radio_RST_GPIO_Port GPIOC
#define RX_LED_GP_Pin GPIO_PIN_7
#define RX_LED_GP_GPIO_Port GPIOC
#define GPIO_1_Pin GPIO_PIN_8
#define GPIO_1_GPIO_Port GPIOA
#define GPIO_2_Pin GPIO_PIN_9
#define GPIO_2_GPIO_Port GPIOA
#define SDIO_NCD_Pin GPIO_PIN_15
#define SDIO_NCD_GPIO_Port GPIOA
#define SPI1_CS_3B7_Pin GPIO_PIN_7
#define SPI1_CS_3B7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
