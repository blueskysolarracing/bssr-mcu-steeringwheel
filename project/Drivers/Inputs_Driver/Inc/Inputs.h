/*
 * Inputs.h
 *
 *  Created on: Oct 14, 2025
 *      Author: Lucas Di Sarra
 */

#ifndef INPUTS_DRIVER_INC_INPUTS_H_
#define INPUTS_DRIVER_INC_INPUTS_H_

#include "stm32l4xx_hal.h"

#define NUM_INPUTS 16

typedef struct
{

	GPIO_TypeDef		*state_gpio_handle;		// ptr to GPIO handle of State read pin

	uint16_t			state_gpio_pin;			// GPIO pin no. of State read pin

	GPIO_TypeDef		*it_gpio_handle;		// ptr to GPIO handle of Interrupt pin

	uint16_t			it_gpio_pin;			// GPIO pin no. of Interrupt pin

	GPIO_TypeDef		**sel_gpio_handles; 	// Array of ptrs to GPIO handles of Selection pins

	uint16_t			*sel_gpio_pins;			// Array of GPIO pin no. of Selection pins

	uint16_t			states;					// List of button states

	uint16_t			states_itmask;			// Mask of states that trigger the interrupt

} Inputs_HandleTypeDef;

uint8_t Inputs_CheckInput(Inputs_HandleTypeDef *inputs, uint8_t input);
uint8_t Inputs_CheckZero(Inputs_HandleTypeDef *inputs);
uint8_t Inputs_CheckAll(Inputs_HandleTypeDef *inputs);

#endif /* INPUTS_DRIVER_INC_INPUTS_H_ */
