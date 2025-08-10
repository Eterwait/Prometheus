#include "mocap_teleop_lib.h"
#include "globals.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "gpio.h"
#include "crc.h"


void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length)
{
	LL_GPIO_SetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Tx mode
	for (int i = 0; i < length; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(USARTx));
		LL_USART_TransmitData8(USARTx, data[i]);
	}
	while (!LL_USART_IsActiveFlag_TC(USARTx))
		;
	LL_USART_ClearFlag_TC(USARTx);
	LL_GPIO_ResetOutputPin(RE_DE_GPIO_Port, RE_DE_Pin); // Set MAX485 to Rx mode
}

uint8_t readForseSensor(void)
{
	memset(force_sensor_4_value, 0, sizeof(force_sensor_4_value)); // Clear force sensors buffer
	memcpy(txBuffer, (uint8_t*)(&command_to_read_force), sizeof(command_to_read_force)); //*
	transmitData(USART2, (uint8_t*)txBuffer, (sizeof(command_to_read_force)));
	LL_mDelay(1);

	for(uint8_t i=0; i<4; i++)
		memcpy(&force_sensor_4_value[i], rxBuffer + (i * sizeof(uint16_t)), sizeof(uint16_t));
	memcpy(&recieved_CRC_32, rxBuffer + (4 * sizeof(uint16_t)), sizeof(uint32_t));

	for (uint8_t i = 0; i < 4; i++)
		CRC_uint32_calc_buffer[i] = (uint32_t)force_sensor_4_value[i];
	calculated_CRC_32 = HAL_CRC_Calculate(&hcrc, &CRC_uint32_calc_buffer, 4);
	if (recieved_CRC_32 == calculated_CRC_32)
		return 1;
	else
		return 0;

}


void clearBufferAndCounter(void)
	{
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxBufferCounter = 0;
		memset(txBuffer, 0, sizeof(txBuffer));
		recieved_CRC_32 = 0;
		memset(CRC_uint32_calc_buffer, 0, sizeof(CRC_uint32_calc_buffer));
	}

void set_new_force(void)
	{
	    if (force_sensor_4_value[0] > 4096)
	    	force_sensor_4_value[0] = 4096;

	    if (force_sensor_4_value[0] < 60)
	    	force_sensor_4_value[0] = 0;

	    pwm_calculations = force_sensor_4_value[3];

	    pwm = (uint16_t)roundf((pwm_calculations * 300.0f) / 4096.0f); // Scale value from 0-4096 to 0-350

	    LL_TIM_OC_SetCompareCH2(TIM3,pwm);
	}

void stop_motor(void)
{
	LL_TIM_OC_SetCompareCH2(TIM3,0);
}

void calibration(void)
	{
		LL_mDelay(100);
		LL_TIM_OC_SetCompareCH2(TIM3,100);
		LL_mDelay(4000);
		TIM4->CNT = 0;
		LL_TIM_OC_SetCompareCH2(TIM3,0);
	}

