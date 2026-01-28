/*
 * hal_common.c
 *
 *  Created on: Jan 21, 2026
 *      Author: Jules
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "hal_common.h"
#include <stdbool.h>

volatile bool fpc2530_irq_active;

static uint32_t button_down_start = 0;
static uint32_t button_down_time = 0;
static bool button_down = false;

uint32_t hal_check_button_pressed(void)
{
	uint32_t button_time = button_down_time;
	if (!button_down) {
		button_down_time = 0;
	}
	return button_down ? 0 : button_time;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_BUTTON_Pin) {
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_SET) {
            button_down_time = HAL_GetTick() - button_down_start;
            button_down = false;
        } else {
            button_down_start = HAL_GetTick();
            button_down = true;
        }
    }
    else if (GPIO_Pin == FPC2530_IRQ_Pin) {
        if (HAL_GPIO_ReadPin(FPC2530_IRQ_GPIO_Port, FPC2530_IRQ_Pin) == GPIO_PIN_SET) {
            fpc2530_irq_active = true;
        }
    }
}

void hal_set_led_status(hal_led_status_t status)
{
	switch(status) {
	        case HAL_LED_STATUS_MATCH:    // Green LED
	        	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	        	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	            break;
	        case HAL_LED_STATUS_NO_MATCH:	// Red LED only
	        	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	        	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	        	break;
	        case HAL_LED_STATUS_WAITTOUCH: // Blue LED only
	        	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				break;
	        case HAL_LED_STATUS_ERROR:    // Toggle all 3 LEDs
	        	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

				HAL_Delay(500);

	        	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	        	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	            break;
	        default:
	        	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	        	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	        	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	    }
}

void hal_reset_device(void)
{
	HAL_GPIO_WritePin(FPC2530_RST_N_GPIO_Port, FPC2530_RST_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(FPC2530_RST_N_GPIO_Port, FPC2530_RST_N_Pin, GPIO_PIN_SET);
}

void hal_set_if_config(hal_if_config_t config)
{
	switch(config) {
		case HAL_IF_CONFIG_UART:
			HAL_GPIO_WritePin(FPC2530_IF_CFG_1_GPIO_Port, FPC2530_IF_CFG_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FPC2530_IF_CFG_2_GPIO_Port, FPC2530_IF_CFG_2_Pin, GPIO_PIN_RESET);
			break;
		case HAL_IF_CONFIG_SPI:
		default:
			HAL_GPIO_WritePin(FPC2530_IF_CFG_1_GPIO_Port, FPC2530_IF_CFG_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FPC2530_IF_CFG_2_GPIO_Port, FPC2530_IF_CFG_2_Pin, GPIO_PIN_RESET);
			break;
	}
}


