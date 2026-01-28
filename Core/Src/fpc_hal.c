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
#include "hal_common.h"
#include "fpc_api.h"
#include "fpc_hal.h"
#include "stm32f4xx_it.h"
#include "fpc_host_sample.h"

#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart6; // Capteur FPC
extern UART_HandleTypeDef huart3; // Debug channel

#ifndef MIN
    #define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#define DMA_BUF_SIZE   128
#define DMA_TIMEOUT_MS 2
#define MAX_LINE_LEN 100

typedef struct
{
    volatile uint8_t flag;      /* Timeout event flag */
    uint16_t prev_cndtr;        /* Holds previous value of DMA_CNDTR (counts down) */
    uint32_t idle_irq_count;    /* Debug */
    uint32_t rx_complete_count; /* Debug */
    uint32_t rx_half_count;     /* Debug */
} dma_event_t;

bool tx_done = false;
static volatile bool tx_half = false;
static volatile bool uart_error = false;
static volatile bool error = false;
static volatile bool rx_available;
static bool ignore_first_idle_irq = true;

extern volatile bool fpc2530_irq_active;

static uint8_t uart_rx_fifo[DMA_BUF_SIZE];
static dma_event_t dma_uart_rx = { 0, DMA_BUF_SIZE, 0, 0, 0 };

fpc_result_t fpc_hal_init(void)
{
	HAL_StatusTypeDef status;

	/* Clear IDLE Flag */
	__HAL_UART_CLEAR_FLAG(&huart6, UART_IT_IDLE);
	/* Enable UART IDLE interrupt */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	/* Start UART RX */
	status = HAL_UART_Receive_DMA(&huart6, uart_rx_fifo, DMA_BUF_SIZE);
	(void)status;

	hal_set_if_config(HAL_IF_CONFIG_UART);
    return FPC_RESULT_OK;
}

fpc_result_t fpc_hal_tx(uint8_t *data, size_t len, uint32_t timeout, int flush)
{
	int rc = uart_host_transmit(data, len, timeout, flush);
	return rc == 0 ? FPC_RESULT_OK : FPC_RESULT_FAILURE;
}

fpc_result_t fpc_hal_rx(uint8_t *data, size_t len, uint32_t timeout)
{
	int rc = uart_host_receive(data, len, timeout);
	return rc == 0 ? FPC_RESULT_OK : FPC_RESULT_FAILURE;
}

int fpc_hal_data_available(void)
{
	// On vérifie le compteur seulement si le DMA est actif
	if (__HAL_DMA_GET_COUNTER(huart6.hdmarx) != dma_uart_rx.prev_cndtr) {
		return 1;
	}

	return rx_available ? 1 : 0;
}

fpc_result_t fpc_hal_wfi(void)
{
    __WFI();
    return FPC_RESULT_OK;
}

void enable_sensor_low_power(void)
{
    fpc_system_config_t my_cfg;

    // Config Version
    my_cfg.version = 1;
    my_cfg.finger_scan_interval_ms = 34;
    my_cfg.sys_flags = 33;
	my_cfg.uart_delay_before_irq_ms = 1;
	my_cfg.uart_baudrate = 5;
	my_cfg.idfy_max_consecutive_fails = 5;
	my_cfg.idfy_lockout_time_s = 15;
    // CHANGING Idle time after last command, before entering stop mode [ms]
    my_cfg.idle_time_before_sleep_ms = 1000;

    fpc_cmd_system_config_set_request(&my_cfg);
}

void log_print(const char *format, ...)
{
	char buffer[256];
	va_list arglist;

	va_start(arglist, format);
	vsnprintf(buffer, sizeof(buffer), format, arglist);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

	va_end(arglist);
}

#ifdef ENABLE_DEBUG_LOGS
void fpc_sample_logf(const char *format, ...)
{
    va_list arglist;
    char tmp[MAX_LINE_LEN];
	int len;

    va_start(arglist, format);
    //uart_debug_vprintf(format, arglist);
    len = vsnprintf(tmp, MAX_LINE_LEN - 2, format, arglist);
	tmp[len] = '\n';
	(void)HAL_UART_Transmit(&huart3, (uint8_t*)tmp, len + 1, 1000);

    va_end(arglist);
}
#endif

///////////// UART and DMA fonctions & IT /////////////

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart6.Instance) {
        rx_available = true;
        dma_uart_rx.rx_half_count++;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart6.Instance) {
        tx_done = true;
    }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart6.Instance) {
        tx_half = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart6.Instance) {
        //uart_error = true;
        uart_host_rx_data_clear();
    }
}

int uart_host_transmit(uint8_t *data, size_t size, uint32_t timeout, int flush)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t tickstart = HAL_GetTick();

    tx_half = false;
    tx_done = false;
    uart_error = false;

    //HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_SET);
    //HAL_Delay(5);

    status = HAL_UART_Transmit_DMA(&huart6, data, (uint16_t)size);

    if (status != HAL_OK) {
        goto exit;
    }

    while (!tx_done) {
        __WFI();

        if (uart_error) {
            status = HAL_ERROR;
            goto exit;
        }

        if (timeout != 0 && (HAL_GetTick() - tickstart > timeout)) {
            if (tx_half) {
                tickstart = HAL_GetTick();
                tx_half = false;
            } else {
                HAL_UART_DMAStop(&huart6);
                status = HAL_TIMEOUT;
                goto exit;
            }
        }
    }

exit:
    return (status == HAL_OK) ? 0 : -1;
}

int uart_host_receive(uint8_t *data, size_t size, uint32_t timeout)
{
    int rc = 0;
    //HAL_StatuypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();

    error = false;

    /* If no size return OK */
    if (!size) {
        goto exit;
    }

    if (huart6.RxState != HAL_UART_STATE_BUSY_RX) {
        // Assert here
    }

    if (__HAL_DMA_GET_COUNTER(huart6.hdmarx) == dma_uart_rx.prev_cndtr) {
        __WFI();
    }

    /* Check and read from RX FIFO */
    while (size) {
        uint16_t curr_cndtr = __HAL_DMA_GET_COUNTER(huart6.hdmarx);

        if (curr_cndtr != dma_uart_rx.prev_cndtr) {
            uint32_t cur_pos = DMA_BUF_SIZE - curr_cndtr;
            uint32_t prev_pos;
            uint32_t length;

            /* Determine start position in DMA buffer based on previous CNDTR value */
            prev_pos = DMA_BUF_SIZE - dma_uart_rx.prev_cndtr;
            if (prev_pos < cur_pos) {
                length = MIN(cur_pos - prev_pos, size);
            } else {
                /* Copy until end of buffer first */
                length = MIN(DMA_BUF_SIZE - prev_pos, size);
            }
            memcpy(data, &uart_rx_fifo[prev_pos], length);
            data += length;
            size -= length;
            dma_uart_rx.prev_cndtr -= length;
            if (dma_uart_rx.prev_cndtr == 0) {
                dma_uart_rx.prev_cndtr = DMA_BUF_SIZE;
            }
            if (prev_pos > cur_pos) {
                /* Copy from start of buffer */
                length = MIN(cur_pos, size);
                memcpy(data, uart_rx_fifo, length);
                data += length;
                size -= length;
                dma_uart_rx.prev_cndtr -= length;
            }
            if (!size) {
                goto exit;
            }
        }

        __WFI();
        if (error) {
            rc = -1;
            goto exit;
        }
        if (timeout != 0 && (HAL_GetTick() - tickstart > timeout)) {
            rc = -1;
            goto exit;
        }
    }

exit:
	if (__HAL_DMA_GET_COUNTER(huart6.hdmarx) == dma_uart_rx.prev_cndtr) {
		rx_available = false;
    }
    return rc;
}

void uart_host_rx_data_clear()
{
	uint8_t temp;
	while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
		temp = (uint8_t)(huart6.Instance->DR & 0x00FF);
		(void)temp; // Annule le warning "set but not used"
	}
}

void host_uart_irq_handler(void)
{
    /* UART IDLE Interrupt */
	if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE) != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);

        if (ignore_first_idle_irq) {
            ignore_first_idle_irq = false;
        }
        else {
            dma_uart_rx.flag = 1;
            dma_uart_rx.idle_irq_count++;
            if(huart6.hdmarx->XferCpltCallback != NULL)
            {
            	huart6.hdmarx->XferCpltCallback(huart6.hdmarx);
            }
        }
    }
}
