/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

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
#define LORA_DIO3_Pin GPIO_PIN_0
#define LORA_DIO3_GPIO_Port GPIOA
#define LORA_DIO1_Pin GPIO_PIN_1
#define LORA_DIO1_GPIO_Port GPIOA
#define eth_tx_Pin GPIO_PIN_2
#define eth_tx_GPIO_Port GPIOA
#define eth_rx_Pin GPIO_PIN_3
#define eth_rx_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#define LORA_RST_Pin GPIO_PIN_0
#define LORA_RST_GPIO_Port GPIOB
#define LORA_DIO0_Pin GPIO_PIN_1
#define LORA_DIO0_GPIO_Port GPIOB
#define wr_Pin GPIO_PIN_12
#define wr_GPIO_Port GPIOB
#define iic_scl_Pin GPIO_PIN_13
#define iic_scl_GPIO_Port GPIOB
#define iic_sda_Pin GPIO_PIN_14
#define iic_sda_GPIO_Port GPIOB
#define debug_tx_Pin GPIO_PIN_9
#define debug_tx_GPIO_Port GPIOA
#define debug_rx_Pin GPIO_PIN_10
#define debug_rx_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOA
#define eth_at_Pin GPIO_PIN_3
#define eth_at_GPIO_Port GPIOB
#define eth_rst_Pin GPIO_PIN_4
#define eth_rst_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
