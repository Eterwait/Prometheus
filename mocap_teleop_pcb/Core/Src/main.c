/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "globals.h"
#include <stdint.h>
#include "mocap_teleop_lib.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	// Start Tim 4 to work, for encoder mode
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM4);

	// Allow Tim 3 to work, it will produce PWM
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM3);

	// Set En and Dir pins
	LL_GPIO_SetOutputPin(GPIOC, EN_motor_Pin);
	LL_GPIO_ResetOutputPin(GPIOA, DIR_motor_Pin);

	// Set PWM to 0 (equal ZERO current)
	LL_TIM_OC_SetCompareCH2(TIM3,0);

	// Allow USART 2 to work
	NVIC_EnableIRQ(USART2_IRQn);

	//  LL_USART_Enable(USART2);
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_ERROR(USART2);

	LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode

//	gripper_position = TIM4->CCR1;
	calibration();
//	gripper_position = TIM4->CCR1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LL_mDelay(1);
	  switch(USB_Rx_buffer[0])
		  {
			 case 'r':
				 // Function with communication with force sensor pcb
				 if (readForseSensor() == 1)
				 {
					 set_new_force();
					 gripper_position = TIM4->CNT;

					 memcpy(USB_Tx_buffer, (uint8_t*)(&gripper_position), sizeof(gripper_position));
					 memcpy(USB_Tx_buffer + sizeof(gripper_position), (uint8_t*)(&force_sensor_4_value), sizeof(force_sensor_4_value));
					 CDC_Transmit_FS(USB_Tx_buffer, sizeof(gripper_position) + sizeof(force_sensor_4_value));

					 clearBufferAndCounter();
					 memset(USB_Tx_buffer, 0, sizeof(USB_Tx_buffer));  // Clear USB Tx buffer
					 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
				 }
				 else
				 {
					 LL_mDelay(1);
					 if (readForseSensor() == 1)
					 {
						 set_new_force();
						 gripper_position = TIM4->CCR1;

						 memcpy(USB_Tx_buffer, (uint8_t*)(&gripper_position), sizeof(gripper_position));
						 memcpy(USB_Tx_buffer + sizeof(gripper_position), (uint8_t*)(&force_sensor_4_value), sizeof(force_sensor_4_value));
						 CDC_Transmit_FS(USB_Tx_buffer, sizeof(gripper_position) + sizeof(force_sensor_4_value));

						 clearBufferAndCounter();
						 memset(USB_Tx_buffer, 0, sizeof(USB_Tx_buffer));  // Clear USB Tx buffer
						 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
					 }
					 else
					 {
						 gripper_position = TIM4->CCR1;
						 memset(force_sensor_4_value, 0, sizeof(force_sensor_4_value)); // Clear force sensors buffer
						 memcpy(USB_Tx_buffer, (uint8_t*)(&gripper_position), sizeof(gripper_position));
						 memcpy(USB_Tx_buffer + sizeof(gripper_position), (uint8_t*)(&force_sensor_4_value), sizeof(force_sensor_4_value));
						 CDC_Transmit_FS(USB_Tx_buffer, sizeof(gripper_position) + sizeof(force_sensor_4_value));

						 clearBufferAndCounter();
						 memset(USB_Tx_buffer, 0, sizeof(USB_Tx_buffer));  // Clear USB Tx buffer
						 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
					 }
//						 CDC_Transmit_FS(test_usb_buffer, sizeof(test_usb_buffer));
//						 stop_force_control_flag=1;
				 }
				 clearBufferAndCounter();
				 memset(USB_Tx_buffer, 0, sizeof(USB_Tx_buffer));  // Clear USB Tx buffer
				 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer

				 break;

			 case 's':
				 stop_motor();
				 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
				 break;

			 default:
				 if (stop_force_control_flag==1)
					 stop_motor();
				 memset(USB_Rx_buffer, 0, sizeof(USB_Rx_buffer)); // Clear USB Rx buffer
				 clearBufferAndCounter();
				 continue;
		  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
