/*
 * LS032B7DD02.c
 *
 *  Created on: Oct 4, 2025
 *      Author: Lucas Di Sarra
 */

#include "../Inc/LS032B7DD02.h"

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t LS032B7DD02_Send(LS032B7DD02_HandleTypeDef *ls032, uint8_t *pData, uint16_t len) {
	uint8_t ret = 0;

	// Assert the CS high
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_SET);
	ret = HAL_SPI_Transmit(ls032->spi_handle, pData, len, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);
		return ERROR;
	}

	// Release the CS
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);

	return SUCCESS;
}

uint8_t LS032B7DD02_Init(LS032B7DD02_HandleTypeDef *ls032) {
	// EXTMODE pin should already be high, default high
	HAL_Delay(1);
	uint8_t start_cmd[2] = {0x20, 0x00};
	LS032B7DD02_Send(ls032, start_cmd, 2);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ls032->disp_gpio_handle, ls032->disp_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_TIM_PWM_Start(ls032->extcomin_tim_handle, ls032->extcomin_channel);
	HAL_Delay(1);
	return HAL_OK;
}

uint8_t LS032B7DD02_Write(LS032B7DD02_HandleTypeDef *ls032) {
	uint8_t test_cmd[1] = {0b10101010};
	LS032B7DD02_Send(ls032, test_cmd, 1);
}

