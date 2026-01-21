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
 */

#ifndef HAL_COMMON_H
#define HAL_COMMON_H

typedef enum {
    HAL_LED_STATUS_READY = 0,
    HAL_LED_STATUS_MATCH,
    HAL_LED_STATUS_NO_MATCH,
    HAL_LED_STATUS_WAITTOUCH,
    HAL_LED_STATUS_ENROLL,
    HAL_LED_STATUS_DELETE_TEMPLATES,
    HAL_LED_STATUS_ERROR
} hal_led_status_t;

typedef enum {
    HAL_IF_CONFIG_SPI = 0,
    HAL_IF_CONFIG_UART
} hal_if_config_t;

uint32_t hal_check_button_pressed(void);
//uint32_t hal_get_button_press_time(void);

void hal_set_led_status(hal_led_status_t status);

void hal_reset_device(void);
void hal_set_if_config(hal_if_config_t config);

#endif
