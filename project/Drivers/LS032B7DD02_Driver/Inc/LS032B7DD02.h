/*
 * LS032B7DD02.h
 *
 *  Created on: Oct 4, 2025
 *      Author: Lucas Di Sarra
 */

#ifndef LS032B7DD02_DRIVER_LS032B7DD02_H_
#define LS032B7DD02_DRIVER_LS032B7DD02_H_

#include "stm32l4xx_hal.h"

#define LS032_PIXEL_HEIGHT 336
#define LS032_PIXEL_WIDTH  536
#define LS032_VRAM_HEIGHT  44
#define LS032_VRAM_START_Y 2

// STRUCTS
// ------------------------------------------------------------------------------------
typedef struct
{
	SPI_HandleTypeDef 	*spi_handle;	// ptr to SPI_HandleTypeDef which interfaces with the LS032

	GPIO_TypeDef		*cs_gpio_handle;// ptr to GPIO handle of CS line

	uint16_t			cs_gpio_pin;	// GPIO pin no. of CS line

	TIM_HandleTypeDef	*extcomin_tim_handle; // ptr to the extcomin timer

	uint16_t			extcomin_channel;	// TIM channel of extcomin

	GPIO_TypeDef		*extmode_gpio_handle;// ptr to GPIO handle of DISP

	uint16_t			extmode_gpio_pin;	// GPIO pin no. of DISP

	GPIO_TypeDef		*disp_gpio_handle;// ptr to GPIO handle of DISP

	uint16_t			disp_gpio_pin;	// GPIO pin no. of DISP

	uint8_t				*vram;			// ptr to the MCU side copy of VRAM

	uint16_t			vram_len;		// vram length

} LS032B7DD02_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

// Commands
uint8_t LS032B7DD02_Init(LS032B7DD02_HandleTypeDef *ls032);
uint8_t LS032B7DD02_Send(LS032B7DD02_HandleTypeDef *ls032, uint8_t *data, uint16_t len);

// General Drawing
uint8_t LS032B7DD02_Clear(LS032B7DD02_HandleTypeDef *ls032);
uint8_t LS032B7DD02_Fill(LS032B7DD02_HandleTypeDef *ls032);
uint8_t LS032B7DD02_Update(LS032B7DD02_HandleTypeDef *ls032);
uint8_t LS032B7DD02_Wipe(LS032B7DD02_HandleTypeDef *ls032);

// Specific Drawing
uint8_t LS032B7DD02_DrawLogo(LS032B7DD02_HandleTypeDef *ls032);

#endif /* LS032B7DD02_DRIVER_LS032B7DD02_H_ */
