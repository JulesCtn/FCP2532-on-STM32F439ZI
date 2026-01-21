/*
 * fpc_hal.c
 *
 *  Created on: Jan 21, 2026
 *      Author: Jules
 */


/**
 * @file    fpc_hal.c
 * @brief   Implementation of the fpc_hal API.
 */

#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "hal_common.h"
#include "fpc_api.h"
#include "fpc_hal.h"

extern UART_HandleTypeDef huart6; // Capteur FPC
extern UART_HandleTypeDef huart3; // Debug channel

fpc_result_t fpc_hal_init(void)
{
    //hal_set_if_config(HAL_IF_CONFIG_UART);
    return FPC_RESULT_OK;
}

fpc_result_t fpc_hal_tx(uint8_t *data, size_t len, uint32_t timeout, int flush)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, data, (uint16_t)len, timeout);
	return (status == HAL_OK) ? FPC_RESULT_OK : FPC_RESULT_FAILURE;
}

fpc_result_t fpc_hal_rx(uint8_t *data, size_t len, uint32_t timeout)
{
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, data, (uint16_t)len, timeout);
	   return (status == HAL_OK) ? FPC_RESULT_OK : FPC_RESULT_FAILURE;
}

int fpc_hal_data_available(void)
{
	return (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET);
}

fpc_result_t fpc_hal_wfi(void)
{
    __WFI();
    return FPC_RESULT_OK;
}

void fpc_sample_logf(const char *format, ...)
{
	char buffer[256];
	va_list arglist;

	va_start(arglist, format);
	vsnprintf(buffer, sizeof(buffer), format, arglist);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

	va_end(arglist);
}
