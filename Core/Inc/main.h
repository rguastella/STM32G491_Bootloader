/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INP_LATCH_Pin GPIO_PIN_13
#define INP_LATCH_GPIO_Port GPIOC
#define BRD_ID0_Pin GPIO_PIN_14
#define BRD_ID0_GPIO_Port GPIOC
#define BRD_ID1_Pin GPIO_PIN_15
#define BRD_ID1_GPIO_Port GPIOC
#define ISEN1_Pin GPIO_PIN_0
#define ISEN1_GPIO_Port GPIOC
#define ISEN2_Pin GPIO_PIN_1
#define ISEN2_GPIO_Port GPIOC
#define ISEN3_Pin GPIO_PIN_2
#define ISEN3_GPIO_Port GPIOC
#define ISEN4_Pin GPIO_PIN_3
#define ISEN4_GPIO_Port GPIOC
#define ISEN5_Pin GPIO_PIN_0
#define ISEN5_GPIO_Port GPIOA
#define ISEN6_Pin GPIO_PIN_1
#define ISEN6_GPIO_Port GPIOA
#define ISEN7_Pin GPIO_PIN_2
#define ISEN7_GPIO_Port GPIOA
#define ISEN8_Pin GPIO_PIN_3
#define ISEN8_GPIO_Port GPIOA
#define ISEN9_Pin GPIO_PIN_6
#define ISEN9_GPIO_Port GPIOA
#define ISEN10_Pin GPIO_PIN_7
#define ISEN10_GPIO_Port GPIOA
#define FLASH_NSS_Pin GPIO_PIN_4
#define FLASH_NSS_GPIO_Port GPIOC
#define ISEN11_Pin GPIO_PIN_5
#define ISEN11_GPIO_Port GPIOC
#define ISEN12_Pin GPIO_PIN_1
#define ISEN12_GPIO_Port GPIOB
#define ISEN13_Pin GPIO_PIN_2
#define ISEN13_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_13
#define CAN_TX_GPIO_Port GPIOB
#define ISEN14_Pin GPIO_PIN_14
#define ISEN14_GPIO_Port GPIOB
#define TP5_Pin GPIO_PIN_15
#define TP5_GPIO_Port GPIOB
#define OUTP_OE_Pin GPIO_PIN_7
#define OUTP_OE_GPIO_Port GPIOC
#define OUTP_LATCH_Pin GPIO_PIN_8
#define OUTP_LATCH_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TP6_Pin GPIO_PIN_15
#define TP6_GPIO_Port GPIOA
#define SPI_CLK_Pin GPIO_PIN_10
#define SPI_CLK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_12
#define SPI_MOSI_GPIO_Port GPIOC
#define EEPROM_NSS_Pin GPIO_PIN_2
#define EEPROM_NSS_GPIO_Port GPIOD
#define CAN_RX_Pin GPIO_PIN_5
#define CAN_RX_GPIO_Port GPIOB
#define INP_NSS_Pin GPIO_PIN_9
#define INP_NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
