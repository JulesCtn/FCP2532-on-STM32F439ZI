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
#include <stdbool.h>

#include "hal_common.h"
#include "fpc_api.h"
#include "fpc_hal.h"

extern UART_HandleTypeDef huart6; // Capteur FPC
extern UART_HandleTypeDef huart3; // Debug channel

#define DMA_BUF_SIZE   128
#define DMA_TIMEOUT_MS 2

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

	if (status != HAL_OK) {
		fpc_sample_logf("DMA RX Start Failed! Status: %d\r\n", status);
	}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6) {
		uint8_t received_byte = uart_rx_fifo[0];

		fpc_sample_logf("DMA Received: %d\r\n", received_byte);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        tx_done = true;
    }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        tx_half = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        uart_error = true;
        tx_done = true; // Pour sortir de la boucle while en cas d'erreur
    }
}

int uart_host_transmit(uint8_t *data, size_t size, uint32_t timeout, int flush)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint32_t tickstart = HAL_GetTick();

    tx_half = false;
    tx_done = false;
    uart_error = false;

    HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

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

    if (__HAL_DMA_GET_COUNTER(&uart6.hdmarx) == dma_uart_rx.prev_cndtr) {
        __WFI();
    }

    /* Check and read from RX FIFO */
    while (size) {
        uint16_t curr_cndtr = __HAL_DMA_GET_COUNTER(&uart_host_hdma_rx);

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
    if (__HAL_DMA_GET_COUNTER(&uart_host_hdma_rx) == dma_uart_rx.prev_cndtr) {
        rx_available = false;
    }
    return rc;
}

void host_uart_irq_handler(void)
{
    /* UART IDLE Interrupt */
    if ((huart6.Instance->SR & USART_ISR_IDLE) != RESET) {
        huart6.Instance->ICR = UART_CLEAR_IDLEF;
        if (ignore_first_idle_irq) {
            ignore_first_idle_irq = false;
        }
        else {
            dma_uart_rx.flag = 1;
            dma_uart_rx.idle_irq_count++;
            if(uart_host_hdma_rx.XferCpltCallback != NULL) {
                uart_host_hdma_rx.XferCpltCallback(&uart_host_hdma_rx);
            }
        }
    }
}
