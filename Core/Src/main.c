/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"

#include "hal_common.h"

#include "fpc_api.h"
#include "fpc_hal.h"
#include "fpc_host_sample.h"

#define N_FINGERS_TO_ENROLL 2

/* Application states */
typedef enum {
    APP_STATE_WAIT_READY = 0,
    APP_STATE_WAIT_VERSION,
    APP_STATE_WAIT_LIST_TEMPLATES,
    APP_STATE_WAIT_ENROLL,
    APP_STATE_WAIT_IDENTIFY,
    APP_STATE_WAIT_ABORT,
    APP_STATE_WAIT_DELETE_TEMPLATES
} app_state_t;

static int quit = 0;
/* Current application state */
static app_state_t app_state = APP_STATE_WAIT_READY;
/* Set after device ready status is received */
static int device_ready = 0;
/* Set after version command response is received */
static int version_read = 0;
/* Set after list templates command response is received */
static int list_templates_done = 0;
/* Updated at each status event from device */
static uint16_t device_state = 0;
/* Set after list templates command response is received */
static int n_templates_on_device = 0;

/* Number of fingers left to enroll */
static int n_fingers_to_enroll = N_FINGERS_TO_ENROLL;

static const fpc_cmd_callbacks_t cmd_cb = {
    .on_error = on_error,
    .on_status = on_status,
    .on_version = on_version,
    .on_enroll = on_enroll,
    .on_identify = on_identify,
    .on_list_templates = on_list_templates
};

static void process_state(void)
{
    app_state_t next_state = app_state;

    switch(app_state) {
        case APP_STATE_WAIT_READY:
            if (device_ready) {
                next_state = APP_STATE_WAIT_VERSION;
                fpc_cmd_version_request();
            }
            break;
        case APP_STATE_WAIT_VERSION:
            if (version_read) {
                next_state = APP_STATE_WAIT_LIST_TEMPLATES;
                fpc_cmd_list_templates_request();
            }
            break;
        case APP_STATE_WAIT_LIST_TEMPLATES:
            if (list_templates_done) {
                if (n_templates_on_device < N_FINGERS_TO_ENROLL) {
                    fpc_id_type_t id_type = {ID_TYPE_GENERATE_NEW, 0};
                    n_fingers_to_enroll = N_FINGERS_TO_ENROLL - n_templates_on_device;
                    fpc_sample_logf("\nStarting enroll %d fingers\n", n_fingers_to_enroll);
                    next_state = APP_STATE_WAIT_ENROLL;
                    fpc_cmd_enroll_request(&id_type);
                }
                else {
                    fpc_id_type_t id_type = {ID_TYPE_ALL, 0};
                    fpc_sample_logf("\nStarting identify\n");
                    next_state = APP_STATE_WAIT_IDENTIFY;
                    fpc_cmd_identify_request(&id_type, 0);
                }
            }
            break;
        case APP_STATE_WAIT_ENROLL:
            if ((device_state & STATE_ENROLL) == 0) {
            	fpc_sample_logf("\nEnroll one finger done.\n");
                n_fingers_to_enroll--;
                if (n_fingers_to_enroll > 0) {
                    fpc_id_type_t id_type = {ID_TYPE_GENERATE_NEW, 0};
                    fpc_sample_logf("\nStarting enroll\n");
                    fpc_cmd_enroll_request(&id_type);
                } else {
                    fpc_id_type_t id_type = {ID_TYPE_ALL, 0};
                    fpc_sample_logf("\nStarting identify\n");
                    next_state = APP_STATE_WAIT_IDENTIFY;
                    fpc_cmd_identify_request(&id_type, 0);
                }
            }
            break;
        case APP_STATE_WAIT_IDENTIFY:
            if ((device_state & STATE_IDENTIFY) == 0) {
                fpc_id_type_t id_type = {ID_TYPE_ALL, 0};
                HAL_Delay(100);
                fpc_cmd_identify_request(&id_type, 0);
            }
            break;
        case APP_STATE_WAIT_ABORT:
            if ((device_state & (STATE_ENROLL | STATE_IDENTIFY)) == 0) {
                fpc_id_type_t id_type = {ID_TYPE_ALL, 0};
                fpc_sample_logf("\nDeleting templates.\n");
                next_state = APP_STATE_WAIT_DELETE_TEMPLATES;
                fpc_cmd_delete_template_request(&id_type);
            }
            break;
        // Will run after next status event is received in response to delete template request.
        case APP_STATE_WAIT_DELETE_TEMPLATES:
        {
            fpc_id_type_t id_type = {ID_TYPE_GENERATE_NEW, 0};
            n_fingers_to_enroll = N_FINGERS_TO_ENROLL;
            hal_set_led_status(HAL_LED_STATUS_WAITTOUCH);
            fpc_sample_logf("\nStarting enroll.\n");
            next_state = APP_STATE_WAIT_ENROLL;
            fpc_cmd_enroll_request(&id_type);
            break;
        }
        default:
            break;
    }

    if (next_state != app_state) {
    	fpc_sample_logf("State transition %d -> %d\n", app_state, next_state);
        app_state = next_state;
    }
}

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  fpc_result_t result;

  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  /* Initialisation SDK FPC */
  fpc_hal_init();
  fpc_host_sample_init((fpc_cmd_callbacks_t*)&cmd_cb);
  hal_reset_device();
  HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  fpc_sample_logf("FPC2532 example app (UART)\r\n");

  // Start by waiting for device status (APP_FW_RDY).
  // All state handling is done in the process_state function.

  // Run through supported commands
  while (1)
  {
	  // Wait for device IRQ or button
	  fpc_hal_wfi();

	  if (hal_check_button_pressed() > 200) {
		  fpc_sample_logf("Button pressed\r\n");
		  app_state = APP_STATE_WAIT_ABORT;
		  fpc_cmd_abort();
	  }

	  if (fpc_hal_data_available()) {
		  result = fpc_host_sample_handle_rx_data();
		  if (result != FPC_RESULT_OK && result != FPC_PENDING_OPERATION) {
			  fpc_sample_logf("Bad incoming data (%d). Wait and try again in some sec\r\n", result);
			  HAL_Delay(100);
			  uart6_host_rx_data_clear();
		  }
		  process_state();
	  }
  }
}

/* Command callbacks */
void on_error(uint16_t error)
{
    hal_set_led_status(HAL_LED_STATUS_ERROR);
    fpc_sample_logf("Got error %d.\r\n", error);
    quit = 1;
}

void on_status(uint16_t event, uint16_t state)
{
    if (state & STATE_APP_FW_READY) {
        device_ready = 1;
    }
    device_state = state;
}

void on_version(char* version)
{
	fpc_sample_logf("Got version: %s\r\n", version);
    version_read = 1;
}

void on_enroll(uint8_t feedback, uint8_t samples_remaining)
{
    extern char *get_enroll_feedback_str_(uint8_t feedback);
    fpc_sample_logf("Enroll samples remaining: %d, feedback: %s (%d)\r\n", samples_remaining,
        get_enroll_feedback_str_(feedback), feedback);
}

void on_identify(int is_match, uint16_t id)
{
    if (is_match) {
        hal_set_led_status(HAL_LED_STATUS_MATCH);
        fpc_sample_logf("Identify match on id %d\r\n", id);
    }
    else {
        hal_set_led_status(HAL_LED_STATUS_NO_MATCH);
        fpc_sample_logf("Identify no match\r\n");
    }
}

void on_list_templates(int num_templates, uint16_t *template_ids)
{
	fpc_sample_logf("Found %d template(s) on device\r\n", num_templates);

    list_templates_done = 1;
    n_templates_on_device = num_templates;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}

void uart6_host_rx_data_clear()
{
	uint8_t temp;
	while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
		temp = (uint8_t)(huart6.Instance->DR & 0x00FF);
		(void)temp; // Annule le warning "set but not used"
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FPC2530_IF_CFG_1_GPIO_Port, FPC2530_IF_CFG_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FPC2530_IF_CFG_2_Pin|FPC2530_RST_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FPC2530_CS_N_GPIO_Port, FPC2530_CS_N_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FPC2530_IF_CFG_1_Pin */
  GPIO_InitStruct.Pin = FPC2530_IF_CFG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FPC2530_IF_CFG_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FPC2530_IRQ_Pin */
  GPIO_InitStruct.Pin = FPC2530_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPC2530_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FPC2530_IF_CFG_2_Pin FPC2530_RST_N_Pin */
  GPIO_InitStruct.Pin = FPC2530_IF_CFG_2_Pin|FPC2530_RST_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FPC2530_CS_N_Pin */
  GPIO_InitStruct.Pin = FPC2530_CS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FPC2530_CS_N_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
