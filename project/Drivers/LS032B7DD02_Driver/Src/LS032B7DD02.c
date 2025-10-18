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

	gate_addr = LS032_PIXEL_WIDTH - gate_addr;			// gate is 1 indexed (FLIPPED)
	//gate_addr++;										// gate is 1 indexed (NORMAL)
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

uint8_t get_idx_from_pos(uint16_t pos_x, uint16_t pos_y, uint16_t *vram_idx) {
	if (pos_x >= LS032_PIXEL_WIDTH) return ERROR;
	if (pos_y >= LS032_VRAM_HEIGHT - 2) return ERROR;

	*vram_idx = pos_x * 44 + pos_y + 2;

	return SUCCESS;
}

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t LS032_Init(LS032_HandleTypeDef *ls032) {
	// EXTMODE pin should already be default high
	// Initialize vram
	memset(ls032->vram, 0x00, ls032->vram_len);
	for (uint16_t col = 0; col < LS032_PIXEL_WIDTH; col++) {
		// Fill in addressing of every column as single array
		get_gate_addr(col, ls032->vram + col*LS032_VRAM_HEIGHT);
	}

	// Initialize register RAM
	for (uint8_t reg = 0; reg < LS032_NUMREGISTERS; reg++) {
		ls032->registers[reg].pos_x = 0;
		ls032->registers[reg].pos_y = 0;
		ls032->registers[reg].size = 0;
		ls032->registers[reg].mode = 0;
		ls032->registers[reg].len = 0;
		memset(ls032->registers[reg].str, 0x00, 0xFF);
	}

	// Flag SPI as Idle
	ls032->spi_state = 0;
	ls032->update_queued = 0;

	delay_us(30);
	// Need to clear twice for some reason
	//LS032_Wipe(ls032);
	//LS032_Wipe(ls032);
	delay_us(30);
	HAL_GPIO_WritePin(ls032->disp_gpio_handle, ls032->disp_gpio_pin, GPIO_PIN_SET);
	delay_us(30);
	HAL_TIM_PWM_Start(ls032->extcomin_tim_handle, ls032->extcomin_channel);
	delay_us(30);
	return SUCCESS;
}

uint8_t LS032_TX_DMA(LS032_HandleTypeDef *ls032, uint8_t *pData, uint16_t len) {
	if (ls032->spi_state != 0) return ERROR;

	// Clear update queue
	ls032->update_queued = 0;

	uint8_t ret = 0;

	// Assert the CS high
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_SET);
	delay_us(3);
	ret = HAL_SPI_Transmit_DMA(ls032->spi_handle, pData, len);
	ls032->spi_state = 1; // Flag SPI as BUSY
	delay_us(1);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);
		ls032->spi_state = 0;
		return ret;
	}

	return SUCCESS;
}

uint8_t LS032_TX_DMA_CPLT(LS032_HandleTypeDef *ls032) {
	// Release the CS
	ls032->spi_state = 0;
	HAL_GPIO_WritePin(ls032->cs_gpio_handle, ls032->cs_gpio_pin, GPIO_PIN_RESET);

	return SUCCESS;
}

uint8_t LS032_TextReg_SetPos(LS032_HandleTypeDef *ls032, uint8_t reg, uint16_t pos_x, uint16_t pos_y) {
	if (reg >= LS032_NUMREGISTERS) 		return ERROR;
	if (pos_x >= LS032_PIXEL_WIDTH) 	return ERROR;
	if (pos_y >= LS032_VRAM_HEIGHT - 2)	return ERROR;

	ls032->registers[reg].pos_x = pos_x;
	ls032->registers[reg].pos_y = pos_y;

	ls032->update_queued = 1;
	return SUCCESS;
}

uint8_t LS032_TextReg_SetSize(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t size) {
	if (reg >= LS032_NUMREGISTERS) return ERROR;
	if (size >= NUM_ALPHNUM_SIZES) return ERROR;

	ls032->registers[reg].size = size;

	ls032->update_queued = 1;
	return SUCCESS;
}

uint8_t LS032_TextReg_SetMode(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t mode) {
	if (reg >= LS032_NUMREGISTERS) return ERROR;

	ls032->registers[reg].mode = mode;

	ls032->update_queued = 1;
	return SUCCESS;
}

uint8_t LS032_TextReg_SetString(LS032_HandleTypeDef *ls032, uint8_t reg, uint8_t len, char* str) {
	if (reg >= LS032_NUMREGISTERS) return ERROR;

	ls032->registers[reg].len = len;
	memset(ls032->registers[reg].str, 0x00, 0xFF);	// Clear contents of string in case len doesnt match
	memcpy(ls032->registers[reg].str, str, len);	// copy str into register buffer

	ls032->update_queued = 1;
	return SUCCESS;
}

// GENERAL DRAWING
// ------------------------------------------------------------------------------------------

uint8_t LS032_UpdateManual(LS032_HandleTypeDef *ls032) {
	if (LS032_DrawScene(ls032)) return ERROR;
	return LS032_TX_DMA(ls032, ls032->vram, ls032->vram_len);
}

uint8_t LS032_UpdateAsync(LS032_HandleTypeDef *ls032) {
	if (ls032->update_queued == 0) return ERROR;
	if (LS032_DrawScene(ls032)) return ERROR;
	return LS032_TX_DMA(ls032, ls032->vram, ls032->vram_len);
}

uint8_t LS032_Wipe(LS032_HandleTypeDef *ls032) {
	uint8_t clear_cmd[2] = {0x20, 0x00};
	return LS032_TX_DMA(ls032, clear_cmd, 2);
}

uint8_t LS032_Clear(LS032_HandleTypeDef *ls032) {
	for (uint16_t x = 0; x < LS032_PIXEL_WIDTH; x++) {
		memset(ls032->vram + x*44 + 2, 0xFF, 42);
	}

	return SUCCESS;
}

uint8_t LS032_Fill(LS032_HandleTypeDef *ls032) {
	for (uint16_t x = 0; x < LS032_PIXEL_WIDTH; x++) {
		memset(ls032->vram + x*44 + 2, 0x00, 42);
	}

	return SUCCESS;
}

uint8_t LS032_DrawRegister(LS032_HandleTypeDef *ls032, uint8_t reg) {
	if (reg >= LS032_NUMREGISTERS) return ERROR;

	return LS032_DrawString(ls032,
			ls032->registers[reg].pos_x,
			ls032->registers[reg].pos_y,
			ls032->registers[reg].size,
			ls032->registers[reg].len,
			ls032->registers[reg].str);
}

uint8_t LS032_DrawScene(LS032_HandleTypeDef *ls032) {
	for (uint8_t reg = 0; reg < LS032_NUMREGISTERS; reg++) {
		if (LS032_DrawRegister(ls032, reg))
			return ERROR;
	}

	return SUCCESS;
}

// SPECIFIC DRAWING
// ------------------------------------------------------------------------------------------

uint8_t LS032_DrawLogo(LS032_HandleTypeDef *ls032) {
	LS032_Clear(ls032);
	uint16_t x_off = (LS032_PIXEL_WIDTH - 500) / 2;
	uint16_t y_off = (42 - 21) / 2;
	for (uint16_t x = 0; x < 500; x++) {
		memcpy(ls032->vram + (x+x_off)*44 + y_off + 2, BSSR_LOGO + x*21, 21);
	}

	return SUCCESS;
}

uint8_t LS032_DrawChar(LS032_HandleTypeDef *ls032, uint16_t pos_x, uint16_t pos_y, uint8_t size, char ch) {
	// Account for newline
	if (ch == '\n') {
		// TODO: Move cursor to newline
		return SUCCESS;
	}

	uint16_t char_idx = ALPHNUM_SIZES_IDX[size][(uint8_t)ch];
	if (char_idx == 0)
		return ERROR;	// char is unable to be rendered

	uint8_t char_width = ALPHNUM_SIZES[size][char_idx];
	uint8_t char_height = ALPHNUM_HEIGHTS[size];
	uint16_t vram_idx = 0;
	get_idx_from_pos(pos_x, pos_y, &vram_idx);

	// Get distance to edge of screen on X
	if (LS032_PIXEL_WIDTH - pos_x < char_width)
		char_width = LS032_PIXEL_WIDTH - pos_x;

	// Get distance to edge of screen on Y
	if (LS032_PIXEL_HEIGHT - pos_y < char_height)
		char_height = LS032_PIXEL_HEIGHT - pos_y;

	for (uint8_t col = 0; col < char_width; col++) {
		memcpy(ls032->vram + vram_idx, ALPHNUM_SIZES[size] + char_idx + 1 + col*char_height, char_height);
		vram_idx += LS032_VRAM_HEIGHT;
	}
	return SUCCESS;
}

uint8_t LS032_DrawString(LS032_HandleTypeDef *ls032, uint16_t pos_x, uint16_t pos_y, uint8_t size, uint8_t len, char* str) {
	for (uint8_t i = 0; i < len; i++) {
		LS032_DrawChar(ls032, pos_x, pos_y, size, str[i]);
		uint16_t char_idx = ALPHNUM_SIZES_IDX[size][(uint8_t)(str[i])];
		pos_x += ALPHNUM_SIZES[size][char_idx];
	}

	return SUCCESS;
}
