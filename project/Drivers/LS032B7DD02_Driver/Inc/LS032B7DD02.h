/*
 * LS032B7DD02.h
 *
 *  Created on: Oct 4, 2025
 *      Author: Lucas Di Sarra
 */

#ifndef LS032B7DD02_DRIVER_LS032B7DD02_H_
#define LS032B7DD02_DRIVER_LS032B7DD02_H_

#include "stm32l4xx_hal.h"

// STRUCTS
// ------------------------------------------------------------------------------------
typedef struct
{
	SPI_HandleTypeDef 	*spi_handle;	// ptr to SPI_HandleTypeDef which interfaces with the LS032

	GPIO_TypeDef		*cs_gpio_handle;// ptr to GPIO handle of CS line

	uint16_t			cs_gpio_pin;	// GPIO pin no. of CS line

	TIM_HandleTypeDef	*extcomin_tim_handle; // ptr to the extcomin timer

	uint16_t			extcomin_channel;	// TIM channel of extcomin

	GPIO_TypeDef		*disp_gpio_handle;// ptr to GPIO handle of DISP

	uint16_t			disp_gpio_pin;	// GPIO pin no. of DISP

} LS032B7DD02_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

uint8_t LS032B7DD02_Send(LS032B7DD02_HandleTypeDef *ls032, uint8_t *data, uint16_t len);
//	uint8_t ST7789_SendByte_Data(ST7789_HandleTypeDef *hst7789, uint8_t data);
//	uint8_t ST7789_SendWord_Data(ST7789_HandleTypeDef *hst7789, uint16_t data);

uint8_t LS032B7DD02_Init(LS032B7DD02_HandleTypeDef *ls032);
uint8_t LS032B7DD02_Write(LS032B7DD02_HandleTypeDef *ls032);

#endif /* LS032B7DD02_DRIVER_LS032B7DD02_H_ */
