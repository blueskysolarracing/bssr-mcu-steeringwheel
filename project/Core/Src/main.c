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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LS032B7DD02.h"
#include "inputs.h"
#include <stdio.h>
#include <string.h>
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

// LS032 Memory allocations
LS032_HandleTypeDef ls032;
uint8_t ls032_vram[LS032_VRAM_HEIGHT*LS032_PIXEL_WIDTH + 2] = {0};
uint16_t ls032_vram_len = LS032_VRAM_HEIGHT*LS032_PIXEL_WIDTH + 2;

LS032_TextReg ls032_registers[LS032_NUMREGISTERS];
char ls032_registers_text[LS032_NUMREGISTERS][0xFF];

// INPUTS Memory allocations
Inputs_HandleTypeDef inputs;
GPIO_TypeDef* input_sel_gpio_ports[4] = {
		INPUT_B0_GPIO_Port,
		INPUT_B1_GPIO_Port,
		INPUT_B2_GPIO_Port,
		INPUT_B3_GPIO_Port
};

uint16_t input_sel_gpio_pins[4] = {
		INPUT_B0_Pin,
		INPUT_B1_Pin,
		INPUT_B2_Pin,
		INPUT_B3_Pin
};

// SPI RX memory allocations
uint8_t spi1_rx_buf[257] = {0};

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
	HAL_GPIO_WritePin(DISPLAY_EXTMODE_GPIO_Port, DISPLAY_EXTMODE_Pin, GPIO_PIN_SET);


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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	// FAULT LIGHT PWM:
	TIM4->CCR1 = 500;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	// READ LIGHT PWM:
	TIM4->CCR3 = 5000;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	// LEFT IND:
	TIM3->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	// RIGHT IND:
	TIM3->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


	// SET UP THE DISPLAY
	ls032.spi_handle = &hspi3;
	ls032.cs_gpio_handle = SPI3_CS_GPIO_Port;
	ls032.cs_gpio_pin = SPI3_CS_Pin;
	ls032.extcomin_tim_handle = &htim2;
	ls032.extcomin_channel = TIM_CHANNEL_2;
	ls032.extmode_gpio_handle = DISPLAY_EXTMODE_GPIO_Port;
	ls032.extmode_gpio_pin = DISPLAY_EXTMODE_Pin;
	ls032.disp_gpio_handle = DISPLAY_DISP_GPIO_Port;
	ls032.disp_gpio_pin = DISPLAY_DISP_Pin;
	ls032.vram = ls032_vram;
	ls032.vram_len = ls032_vram_len;

	// Assign register memory to references in LS032
	ls032.registers = ls032_registers;
	for (uint8_t i = 0; i < 32; i++)
		ls032.registers[i].str = ls032_registers_text[i];

	if (LS032_Init(&ls032)) {
		// TODO: Error Handle
	}

	// SET UP THE INPUTS LIBRARY
	inputs.sel_gpio_handles = input_sel_gpio_ports;
	inputs.sel_gpio_pins = input_sel_gpio_pins;
	inputs.it_gpio_handle = INPUT_IT_GPIO_Port;
	inputs.it_gpio_pin = INPUT_IT_Pin;
	inputs.state_gpio_handle = INPUT_STATE_GPIO_Port;
	inputs.state_gpio_pin = INPUT_STATE_Pin;
	inputs.states = 0x0000;
	inputs.states_itmask = 0xFFFF;

	LS032_DrawLogo(&ls032);
	//LS032_Update(&ls032);

	uint8_t tmp_num = 0;
	char speed_letters[255];
	char speed_bars_1[255];
	char speed_bars_2[255];
	char *speed_units = "KM/H";
	char inputs_vis[255];

	memset(speed_bars_1, '/', 255);
	memset(speed_bars_2, '\\', 255);

	LS032_TextReg_SetPos(&ls032, 0x02, 380, 26);
	LS032_TextReg_SetSize(&ls032, 0x02, 3);

	LS032_TextReg_SetString(&ls032, 0x03, strlen(speed_units), speed_units);
	LS032_TextReg_SetPos(&ls032, 0x03, 420, 24);
	LS032_TextReg_SetSize(&ls032, 0x03, 1);

	LS032_TextReg_SetPos(&ls032, 0x00, 0, 30);
	LS032_TextReg_SetSize(&ls032, 0x00, 1);
	LS032_TextReg_SetPos(&ls032, 0x01, 0, 34);
	LS032_TextReg_SetSize(&ls032, 0x01, 1);

	LS032_TextReg_SetPos(&ls032, 0x04, 00, 0);
	LS032_TextReg_SetSize(&ls032, 0x04, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  sprintf(speed_letters, "%d", tmp_num);

	  LS032_Clear(&ls032);

	  LS032_TextReg_SetString(&ls032, 0x02, strlen(speed_letters), speed_letters);

	  LS032_TextReg_SetPos(&ls032, 0x00, (tmp_num % 5)*4, 30);
	  LS032_TextReg_SetPos(&ls032, 0x01, (tmp_num % 5)*4, 34);
	  LS032_TextReg_SetString(&ls032, 0x00, tmp_num/5, speed_bars_1);
	  LS032_TextReg_SetString(&ls032, 0x01, tmp_num/5, speed_bars_2);

	  // Check and write the inputs:
	  sprintf(inputs_vis, "%d", Inputs_CheckZero(&inputs));
	  LS032_TextReg_SetString(&ls032, 0x04, strlen(inputs_vis), inputs_vis);


	  LS032_UpdateAsync(&ls032);

	  tmp_num += 1;
	  if (tmp_num > 99)
		  tmp_num = 0;

	  HAL_Delay(30);
//	LS032B7DD02_Clear(&ls032);
//	LS032B7DD02_Update(&ls032);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ------------------------------------------------------------ HELPER FUNCTIONS -- //

void Handle_SPI1_RX_START() {
	// Reset the buffer and start DMA
	memset(spi1_rx_buf, 0x00, 257);
	HAL_SPI_Receive_DMA(&hspi1, spi1_rx_buf, 257);
}

void Handle_SPI1_RX_CPLT() {
	// Stop DMA and parse the packet
	HAL_SPI_DMAStop(&hspi1);

	if (spi1_rx_buf[0] & 0b10000000) {
		// DISPLAY CMD
		uint8_t reg  = (spi1_rx_buf[0] & 0b01111100) >> 2;
		uint8_t prop = (spi1_rx_buf[0] & 0b00000011);
		switch (prop) {
			case 0:
				uint16_t pos_x = (((uint16_t)spi1_rx_buf[1]) << 8) | ((uint16_t)spi1_rx_buf[2]);
				uint16_t pos_y = (((uint16_t)spi1_rx_buf[3]) << 8) | ((uint16_t)spi1_rx_buf[4]);
				LS032_TextReg_SetPos(&ls032, reg, pos_x, pos_y);
				break;
			case 1:
				uint8_t size = spi1_rx_buf[1];
				LS032_TextReg_SetSize(&ls032, reg, size);
				break;
			case 2:
				uint8_t mode = spi1_rx_buf[1];
				LS032_TextReg_SetMode(&ls032, reg, mode);
				break;
			case 3:
				uint8_t len = spi1_rx_buf[1];
				LS032_TextReg_SetString(&ls032, reg, len, (char*)(spi1_rx_buf + 2));
				break;
			default:
				break;
		}
	} else {
		// INPUT CMD
		//TODO: Return values
	}
}

// ------------------------------------------------------------ OVERRIDE EXTERNAL INTERRUPTS -- //
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SPI1_CS_Pin) {
	  if (HAL_GPIO_ReadPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin)) {
		  // SPI CS was just deasserted
		  Handle_SPI1_RX_CPLT();
	  } else {
		  // SPI CS was just asserted
		  Handle_SPI1_RX_START();
	  }
  } else {
      __NOP();
  }
}

// ------------------------------------------------------------ OVERRIDE SPI DMA CALLBACKS -- //
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//	ls032.spi_state = 0;
//	HAL_GPIO_WritePin(ls032.cs_gpio_handle, ls032.cs_gpio_pin, GPIO_PIN_RESET);
	LS032_TX_DMA_CPLT(&ls032);
}

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
#ifdef USE_FULL_ASSERT
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
