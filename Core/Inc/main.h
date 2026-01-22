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
void on_error(uint16_t error);
void on_status(uint16_t event, uint16_t state);
void on_version(char* version);
void on_enroll(uint8_t feedback, uint8_t samples_remaining);
void on_identify(int is_match, uint16_t id);
void on_list_templates(int num_templates, uint16_t *template_ids);
void uart6_host_rx_data_clear();

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USER_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define FPC2530_IF_CFG_1_Pin GPIO_PIN_14
#define FPC2530_IF_CFG_1_GPIO_Port GPIOF
#define FPC2530_IRQ_Pin GPIO_PIN_15
#define FPC2530_IRQ_GPIO_Port GPIOF
#define FPC2530_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define FPC2530_IF_CFG_2_Pin GPIO_PIN_11
#define FPC2530_IF_CFG_2_GPIO_Port GPIOE
#define FPC2530_RST_N_Pin GPIO_PIN_13
#define FPC2530_RST_N_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define FPC2530_CS_N_Pin GPIO_PIN_14
#define FPC2530_CS_N_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
