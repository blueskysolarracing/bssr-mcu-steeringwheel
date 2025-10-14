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

#define LS032_NUMREGISTERS 32

// STRUCTS
// ------------------------------------------------------------------------------------
typedef struct
{
	uint8_t 			idx;

	uint16_t			pos_x;	// vram draw position

	uint16_t			pos_y;

	uint8_t 			size;

	uint8_t				mode;

	uint8_t				len;

	char				*str;

} LS032_TextReg;

typedef struct
{
	SPI_HandleTypeDef 	*spi_handle;			// ptr to SPI_HandleTypeDef which interfaces with the LS032

	GPIO_TypeDef		*cs_gpio_handle;		// ptr to GPIO handle of CS line

	uint16_t			cs_gpio_pin;			// GPIO pin no. of CS line

	uint8_t 			spi_state;				// State of the SPI Transfer. 0 = Idle, 1 = Busy

	uint8_t 			update_queued;			// State of the SPI Queue. 0 = No queue, 1 = Queue

	TIM_HandleTypeDef	*extcomin_tim_handle; 	// ptr to the extcomin timer

	uint16_t			extcomin_channel;		// TIM channel of extcomin

	GPIO_TypeDef		*extmode_gpio_handle;	// ptr to GPIO handle of DISP

	uint16_t			extmode_gpio_pin;		// GPIO pin no. of DISP

	GPIO_TypeDef		*disp_gpio_handle;		// ptr to GPIO handle of DISP

	uint16_t			disp_gpio_pin;			// GPIO pin no. of DISP

	uint8_t				*vram;					// ptr to the MCU side copy of VRAM

	uint16_t			vram_len;				// vram length

	LS032_TextReg		*registers;

} LS032_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

// Commands
uint8_t LS032_Init(LS032_HandleTypeDef *ls032);
uint8_t LS032_TX_DMA(LS032_HandleTypeDef *ls032, uint8_t *data, uint16_t len);
uint8_t LS032_TX_DMA_CPLT(LS032_HandleTypeDef *ls032);

uint8_t LS032_TextReg_SetPos(LS032_HandleTypeDef *ls032, uint8_t reg, uint16_t pos_x, uint16_t pos_y);
uint8_t LS032_TextReg_SetSize(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t size);
uint8_t LS032_TextReg_SetMode(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t mode);
uint8_t LS032_TextReg_SetString(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t len, char* str);

// General Drawing
uint8_t LS032_Clear(LS032_HandleTypeDef *ls032);
uint8_t LS032_Fill(LS032_HandleTypeDef *ls032);
uint8_t LS032_UpdateManual(LS032_HandleTypeDef *ls032);
uint8_t LS032_UpdateAsync(LS032_HandleTypeDef *ls032);
uint8_t LS032_Wipe(LS032_HandleTypeDef *ls032);

uint8_t LS032_DrawRegister(LS032_HandleTypeDef *ls032, uint8_t reg);
uint8_t LS032_DrawScene(LS032_HandleTypeDef *ls032);

// Specific Drawing
uint8_t LS032_DrawLogo(LS032_HandleTypeDef *ls032);
uint8_t LS032_DrawChar(LS032_HandleTypeDef *ls032, uint16_t pos_x, uint16_t pos_y, uint8_t size, char ch);
uint8_t LS032_DrawString(LS032_HandleTypeDef *ls032, uint16_t pos_x, uint16_t pos_y, uint8_t size, uint8_t len, char* str);

#endif /* LS032B7DD02_DRIVER_LS032B7DD02_H_ */
