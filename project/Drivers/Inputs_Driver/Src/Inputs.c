/*
 * Inputs.c
 *
 *  Created on: Oct 14, 2025
 *      Author: Lucas Di Sarra
 */

#include "../Inc/Inputs.h"

uint8_t Inputs_CheckInput(Inputs_HandleTypeDef *inputs, uint8_t input) {
	if (input >= NUM_INPUTS) return ERROR;

	HAL_GPIO_WritePin(inputs->sel_gpio_handles[3], inputs->sel_gpio_pins[3], input & 0b1000);
	HAL_GPIO_WritePin(inputs->sel_gpio_handles[2], inputs->sel_gpio_pins[2], input & 0b0100);
	HAL_GPIO_WritePin(inputs->sel_gpio_handles[1], inputs->sel_gpio_pins[1], input & 0b0010);
	HAL_GPIO_WritePin(inputs->sel_gpio_handles[0], inputs->sel_gpio_pins[0], input & 0b0001);

	uint8_t old_state = inputs->states;

	// Write state changes
	if (HAL_GPIO_ReadPin(inputs->state_gpio_handle, inputs->state_gpio_pin))
		inputs->states |= (0x0001 << input);
	else
		inputs->states &= (0xFFFE << input);

	// Check if the masked buttons have changed
	if ((old_state & inputs->states_itmask) != (inputs->states & inputs->states_itmask))
		HAL_GPIO_WritePin(inputs->it_gpio_handle, inputs->it_gpio_pin, GPIO_PIN_SET);

	return SUCCESS;
}

uint8_t Inputs_CheckAll(Inputs_HandleTypeDef *inputs) {
	for (uint8_t i = 0; i < NUM_INPUTS; i++) {
		Inputs_CheckInput(inputs, i);
	}

	return SUCCESS;
}
