/*
 * Copyright (c) 2024 Fingerprint Cards AB
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/**
 * @file    fpc_hal.h
 * @brief   HAL definitions for SDK example
 *
 * The function prototypes in this file shall be implemented on the target
 * platform.
 *
 */

#ifndef FPC_HAL_H_
#define FPC_HAL_H_

/**
 * @brief Enable debug log printouts by setting this define.
*/
#define ENABLE_DEBUG_LOGS


#ifdef ENABLE_DEBUG_LOGS
/**
 * @brief Debug Log function. Printf style
 */
void fpc_sample_logf(const char *format, ...);
int uart_host_transmit(uint8_t *data, size_t size, uint32_t timeout, int flush);
int uart_host_receive(uint8_t *data, size_t size, uint32_t timeout);
void host_uart_irq_handler(void);
void uart_host_rx_data_clear();
#else
/**
 * @brief Debug Log function when logging is disabled => Void.
 */
#define fpc_sample_logf(...)
#endif


/**
 * @brief HAL Initialization function.
 *
 * This function is called from the fpc_host_sample_init function. If the HAL
 * initialization is already taken care of elsewhere, this function can be made
 * with an empty body, returning FPC_RESULT_OK.
 *
 * @return Result Code
 */
fpc_result_t fpc_hal_init(void);


/**
 * @brief Data Transmit function.
 *
 * The data buffer is allowed to be overwritten by the implementation if that
 * is feasable.
 *
 * @param data Buffer to transmit data from.
 * @param len Length of buffer to transfer
 * @param timeout Timeout value in milliseconds.
 * @param flush Set to 1 to flush data to host (needed for SPI transfers).
 *
 * @return Result Code
 */
fpc_result_t fpc_hal_tx(uint8_t *data, size_t len, uint32_t timeout, int flush);


/**
 * @brief Data Receive function.
 *
 * @param data buffer to receive data to.
 * @param len length of buffer to transfer
 * @param timeout timout value in ms.
 *
 * @return Result Code
 */
fpc_result_t fpc_hal_rx(uint8_t *data, size_t len, uint32_t timeout);


/**
 * @brief Check if the FPS Module has its IRQ signal active or data in rx buffer
 *
 * Active IRQ means Data Available on the FPS Module.
 *
 * @return non-zero if IRQ is high. Zero otherwise.
 */
int fpc_hal_data_available(void);


/**
 * @brief Wait For Interrupt
 *
 * This function is meant to be blocking until there is a system interrupt,
 * including the FPS Module interrupt.
 *
 * @return Result Code
 */
fpc_result_t fpc_hal_wfi(void);


/**
 * @brief Blocking wait function
 *
 * @param ms Time in milliseconds to wait
 */
void fpc_hal_delay_ms(uint32_t ms);

#endif /* FPC_HAL_H_ */
