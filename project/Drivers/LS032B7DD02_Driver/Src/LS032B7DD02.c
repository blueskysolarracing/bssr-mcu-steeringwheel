/*
 * LS032B7DD02.c
 *
 *  Created on: Oct 4, 2025
 *      Author: Lucas Di Sarra
 */

#include "../Inc/LS032B7DD02.h"
#include <Bitmaps.h>
#include <string.h>

// HELPER FUNCS
// ------------------------------------------------------------------------------------
void delay_us(uint32_t us) {
	// 64MHz clock. 1us = 64CLK
	for (uint32_t i = 0; i < us*64; i++) {
		asm("NOP");
	}
}

void get_gate_addr(uint16_t gate_addr, uint8_t *bytes) {

	gate_addr = LS032_PIXEL_WIDTH - gate_addr;			// gate is 1 indexed
	bytes[0] = 0b10000000; 	// Mode select (M0=H, M1=L, M2=L)

	// This shoves 10 bits into B0 and B1, while flipping the endianness
	bytes[0] |= (gate_addr & 0b0000000000000001) << 1;
	bytes[0] |= (gate_addr & 0b0000000000000010) >> 1;
	bytes[1] |= (gate_addr & 0b0000000000000100) << 5;
	bytes[1] |= (gate_addr & 0b0000000000001000) << 3;
	bytes[1] |= (gate_addr & 0b0000000000010000) << 1;
	bytes[1] |= (gate_addr & 0b0000000000100000) >> 1;
	bytes[1] |= (gate_addr & 0b0000000001000000) >> 3;
	bytes[1] |= (gate_addr & 0b0000000010000000) >> 5;
	bytes[1] |= (gate_addr & 0b0000000100000000) >> 7;
	bytes[1] |= (gate_addr & 0b0000001000000000) >> 9;
}

uint8_t get_idx_from_cursor(LS032B7DD02_HandleTypeDef *ls032, uint16_t *vram_idx) {
	if (ls032->cursor_x >= LS032_PIXEL_WIDTH) return ERROR;
	if (ls032->cursor_y >= LS032_VRAM_HEIGHT - 2) return ERROR;

	*vram_idx = ls032->cursor_x * 44 + ls032->cursor_y + 2;

	return SUCCESS;
}

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t LS032B7DD02_Init(LS032B7DD02_HandleTypeDef *ls032) {
	// EXTMODE pin should already be default high
	// Initialize vram
	memset(ls032->vram, 0x00, ls032->vram_len);
	for (uint16_t col = 0; col < LS032_PIXEL_WIDTH; col++) {
		// Fill in addressing of every column as single array
		get_gate_addr(col, ls032->vram + col*LS032_VRAM_HEIGHT);
	}

	delay_us(30);
	// Need to clear twice for some reason
	LS032B7DD02_Wipe(ls032);
	LS032B7DD02_Wipe(ls032);
	delay_us(30);
	HAL_GPIO_WritePin(ls032->disp_gpio_handle, ls032->disp_gpio_pin, GPIO_PIN_SET);
	delay_us(30);
	HAL_TIM_PWM_Start(ls032->extcomin_tim_handle, ls032->extcomin_channel);
	delay_us(30);
	return SUCCESS;
}

uint8_t LS032B7DD02_Send(LS032B7DD02_HandleTypeDef *ls032, uint8_t *pData, uint16_t len) {
	uint8_t ret = 0;

	// Assert the CS high
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_SET);
	delay_us(3);
	ret = HAL_SPI_Transmit(ls032->spi_handle, pData, len, 100);
	delay_us(1);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);
		return ERROR;
	}

	// Release the CS
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);

	return SUCCESS;
}

// GENERAL DRAWING
// ------------------------------------------------------------------------------------------

uint8_t LS032B7DD02_Update(LS032B7DD02_HandleTypeDef *ls032) {
	// Push VRAM to LCD Mem.
	// Todo: Make this a DMA TX
	return LS032B7DD02_Send(ls032, ls032->vram, ls032->vram_len);
}

uint8_t LS032B7DD02_Wipe(LS032B7DD02_HandleTypeDef *ls032) {
	uint8_t clear_cmd[2] = {0x20, 0x00};
	return LS032B7DD02_Send(ls032, clear_cmd, 2);
}

uint8_t LS032B7DD02_Clear(LS032B7DD02_HandleTypeDef *ls032) {
	for (uint16_t x = 0; x < LS032_PIXEL_WIDTH; x++) {
		memset(ls032->vram + x*44 + 2, 0xFF, 42);
	}

	return SUCCESS;
}

uint8_t LS032B7DD02_Fill(LS032B7DD02_HandleTypeDef *ls032) {
	for (uint16_t x = 0; x < LS032_PIXEL_WIDTH; x++) {
		memset(ls032->vram + x*44 + 2, 0x00, 42);
	}

	return SUCCESS;
}

// SPECIFIC DRAWING
// ------------------------------------------------------------------------------------------

uint8_t LS032B7DD02_DrawLogo(LS032B7DD02_HandleTypeDef *ls032) {
	LS032B7DD02_Clear(ls032);
	uint16_t x_off = (LS032_PIXEL_WIDTH - 500) / 2;
	uint16_t y_off = (42 - 21) / 2;
	for (uint16_t x = 0; x < 500; x++) {
		memcpy(ls032->vram + (x+x_off)*44 + y_off + 2, BSSR_LOGO + x*21, 21);
	}

	return SUCCESS;
}

uint8_t LS032B7DD02_DrawChar(LS032B7DD02_HandleTypeDef *ls032, char ch) {
	// Account for newline
	if (ch == '\n') {
		// TODO: Move cursor to newline
		return SUCCESS;
	}

	uint16_t char_idx = ALPHNUM_1_IDX[(uint8_t)ch];
	if (char_idx == 0)
		return ERROR;	// char is unable to be rendered

	uint8_t char_width = ALPHNUM_1[char_idx];
	uint8_t char_height = 1;
	uint16_t vram_idx = 0;
	get_idx_from_cursor(ls032, &vram_idx);

	// Get distance to edge of screen on X
	if (LS032_PIXEL_WIDTH - ls032->cursor_x < char_width)
		char_width = LS032_PIXEL_WIDTH - ls032->cursor_x;

	// Get distance to edge of screen on Y
	if (LS032_PIXEL_HEIGHT - ls032->cursor_y < char_height)
		char_height = LS032_PIXEL_HEIGHT - ls032->cursor_y;

	for (uint8_t col = 0; col < char_width; col++) {
		memcpy(ls032->vram + vram_idx, ALPHNUM_1 + char_idx + 1 + col*char_height, char_height);
		vram_idx += LS032_VRAM_HEIGHT;
	}

	ls032->cursor_x += char_width;
	return SUCCESS;
}

uint8_t LS032B7DD02_DrawString(LS032B7DD02_HandleTypeDef *ls032, uint8_t len, char* str) {
	for (uint8_t i = 0; i < len; i++) {
		LS032B7DD02_DrawChar(ls032, str[i]);
	}

	return SUCCESS;
}
