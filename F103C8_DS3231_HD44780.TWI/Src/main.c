/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "hd44780_twi.h"
#include "ds3231.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t sec = 0, min = 0, hour = 0, day = 0, date = 0, month = 0, year = 0, a1sec = 0, a1min = 0, a1hour = 0, a1day = 0, a1date = 0, a2min = 0, a2hour = 0, a2day = 0, a2date = 0,
power = 0, set = 0, temperatureAfterpoint;
float temperature = 0.0;
uint64_t milliseconds, updateRTC, updateUART;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t rtcBuffer[19];

	uint8_t data[] = "UART OK\n";
	uint8_t rx_index = 0;
	uint8_t rx_data;
	uint8_t rx_buffer[256];
	
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		uint8_t i;
		if (huart->Instance == USART1)
		{
			if (rx_index == 0)
			{
				for (i = 0; i < 255; i++)
				{
					rx_buffer[i] = 0;
				}			
			}		
			rx_buffer[rx_index++] = rx_data;

	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
		}
	}

	void relayLight(void)
{
		if (hour == a1hour && min == a1min) power = 1;
		if (hour == a2hour && min == a2min) power = 0;
		
			if (!power) 
			{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HD44780_SetPos(0, 10);
			HD44780_SendChar(' ');
			HD44780_SetPos(1, 10);
			HD44780_SendChar('>');
			}
			else 
			{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HD44780_SetPos(0, 10);
			HD44780_SendChar('>');
			HD44780_SetPos(1, 10);
			HD44780_SendChar(' ');
			}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_UART_Receive_IT (&huart1, &rx_data, 1);
		
	uint8_t uartTransmit[] = "UART OK\r\n";
	HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);
	
	HD44780_Init();
	HD44780_Clear();
	HD44780_SetPos(0, 0);
	HD44780_String("*** STRING 0 ***");
	HD44780_SetPos(1, 0);
	HD44780_String("*** STRING 1 ***");
	HAL_Delay(500);
	HD44780_Clear();
	
	HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (rx_index != 0)
		{
		HD44780_Clear();
		HD44780_SetPos(0, 0);
		HD44780_String((char*)rx_buffer);
		rx_index = 0;
		HAL_Delay(1000);
		}
		
if (milliseconds > updateRTC + 250)
{
		updateRTC = milliseconds;
		
		rtcBuffer[0] = 0;
		
		HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, rtcBuffer, 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcBuffer, 19, 100);

			if ((bcdToDec(rtcBuffer[0]) % 2 == 0 || set == 1 || set == 2 || set == 3) && (set != 4 && set != 5 && set != 6))
			{
		hour = rtcBuffer[2];
		hour = bcdToDec(hour);
		HD44780_SetPos(0, 0);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 1) HD44780_String("  ");
				else 
				{
		HD44780_SendChar((char) ((hour / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((hour) % 10) + 0x30);
				}
		HD44780_SendChar(':');

		min = rtcBuffer[1];
		min = bcdToDec(min);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 2) HD44780_String("  ");
				else
				{
		HD44780_SendChar((char) ((min / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((min) % 10) + 0x30);
				}
		HD44780_SendChar(':');
				
		sec = rtcBuffer[0];
		sec = bcdToDec(sec);
		HD44780_SendChar((char) ((sec / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((sec) % 10) + 0x30);

		HD44780_SendChar(' ');
		day = rtcBuffer[3];
		day = bcdToDec(day);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 3) HD44780_String(" ");
				else HD44780_SendChar((char) (day) + 0x30);
		HD44780_SetPos(1, 8);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 3) HD44780_String("  ");
				else
				{
	switch(day)
		{
			case 1: HD44780_String("SU"); break;
			case 2: HD44780_String("MO"); break;
			case 3: HD44780_String("TU"); break;
			case 4: HD44780_String("WE"); break;
			case 5: HD44780_String("TH"); break;
			case 6: HD44780_String("FR"); break;
			case 7: HD44780_String("SA"); break;
		}
	}
			}	
				else if ((bcdToDec(rtcBuffer[0]) % 2 != 0 || set == 4 || set == 5 || set == 6) && (set != 1 && set != 2 && set != 3))
			{
		HD44780_SetPos(0, 0);
		date = rtcBuffer[4];
		date = bcdToDec(date);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 4) HD44780_String("  ");
				else
				{
		HD44780_SendChar((char) ((date / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((date) % 10) + 0x30);
				}
		HD44780_SendChar('/');
		month = rtcBuffer[5];
		month = bcdToDec(month);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 5) HD44780_String("  ");
				else
					{
		HD44780_SendChar((char) ((month / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((month) % 10) + 0x30);
					}
		HD44780_SendChar('/');
		year = rtcBuffer[6];
		year = bcdToDec(year);
					if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 6) HD44780_String("  ");
					else
					{
		HD44780_SendChar((char) ((year / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((year) % 10) + 0x30);
					}
		HD44780_SendChar(' ');
			}
			
		a1hour = rtcBuffer[9];
		a1hour = bcdToDec(a1hour);
		HD44780_SetPos(0, 11);
			if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 7) HD44780_String("  ");
			else
				{
		HD44780_SendChar((char) ((a1hour / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((a1hour) % 10) + 0x30);
				}
		HD44780_SendChar(':');
		a1min = rtcBuffer[8];
		a1min = bcdToDec(a1min);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 8) HD44780_String("  ");
				else
				{
		HD44780_SendChar((char) ((a1min / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((a1min) % 10) + 0x30);
				}
		a2hour = rtcBuffer[13];
		a2hour = bcdToDec(a2hour);
		HD44780_SetPos(1, 11);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 9) HD44780_String("  ");
				else
				{
		HD44780_SendChar((char) ((a2hour / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((a2hour) % 10) + 0x30);
				}
		HD44780_SendChar(':');
		a2min = rtcBuffer[12];
		a2min = bcdToDec(a2min);
				if (bcdToDec(rtcBuffer[0]) % 2 == 0 && set == 10) HD44780_String("  ");
				else
				{
		HD44780_SendChar((char) ((a2min / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((a2min) % 10) + 0x30);
				}
				
		temperature = DS3231_getTemperature(rtcBuffer[17], rtcBuffer[18]);
		HD44780_SetPos(1, 0);
		temperatureAfterpoint = ((temperature*100)-(uint8_t)temperature*100);	
		HD44780_SendChar((char) (((uint8_t)temperature / 10) % 10) + 0x30);
		HD44780_SendChar((char) (((uint8_t)temperature) % 10) + 0x30);
		HD44780_SendChar('.');
		HD44780_SendChar((char) ((temperatureAfterpoint / 10) % 10) + 0x30);
		HD44780_SendChar((char) ((temperatureAfterpoint) % 10) + 0x30);
		
		HD44780_SetPos(1, 5);
		HD44780_SendChar(0xDF);
		HD44780_SendChar('C');
		
		if (!sec && (a1hour != a2hour || a1min != a2min)) relayLight();
}

if (milliseconds > updateUART + 5000)
{
		updateUART = milliseconds;
	
		uint8_t uartTransmit[] = "T:";
		HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);
	
		uint8_t uartTransmitTemperature[8] = { NULL };
		uartTransmitTemperature[0] = ((((uint8_t)temperature / 10) % 10) + 0x30);
		uartTransmitTemperature[1] = ((((uint8_t)temperature) % 10) + 0x30);
		uartTransmitTemperature[2] = 0x2E;
		uartTransmitTemperature[3] =  (((temperatureAfterpoint / 10) % 10) + 0x30);
		uartTransmitTemperature[4] = (((temperatureAfterpoint) % 10) + 0x30);
		uartTransmitTemperature[5] = 0x43;
		uartTransmitTemperature[6] = 0x0A;
		uartTransmitTemperature[7] = 0x0D;
	
		HAL_UART_Transmit(&huart1, uartTransmitTemperature, sizeof(uartTransmitTemperature), 100);
}
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
		{
			if (power) power = 0; else power = 1;
			relayLight(); HAL_Delay(500);
		}
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET)
		{
			if (set < 10) set++; else set = 0;
			HAL_Delay(500);
		}
			
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET)
		{
			rtcBuffer[0] = 0;	
			HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, rtcBuffer, 1, 100);
			HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcBuffer, 19, 100);
			rtcBuffer[0] = 0; 
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtcBuffer, 1, 100);
			switch (set)
			{
				case 1: if (hour > 0) hour--; else hour = 23; break;
				case 2: if (min > 0) { min--; sec = 0;} else { min = 59; sec = 0;} break;
				case 3: if (day > 1) day--; else day = 7; break;
				case 4: if (date > 1) date--; else date = 31; break;
				case 5: if (month > 1) month--; else month = 12; break;
				case 6: if (year > 0) year--; else year = 99; break;
				case 7: if (a1hour > 0) a1hour--; else a1hour = 23; break;
				case 8: if (a1min > 0) a1min--; else a1min = 59; break;
				case 9: if (a2hour > 0) a2hour--; else a2hour = 23; break;
				case 10: if (a2min > 0) a2min--; else a2min = 59; break;			
			}
				rtcBuffer[1] = decToBcd(sec);
				rtcBuffer[2] = decToBcd(min);
				rtcBuffer[3] = decToBcd(hour);
				rtcBuffer[4] = decToBcd(day);
				rtcBuffer[5] = decToBcd(date);
				rtcBuffer[6] = decToBcd(month);
				rtcBuffer[7] = decToBcd(year);	
				rtcBuffer[8] = decToBcd(a1sec);
				rtcBuffer[9] = decToBcd(a1min);
				rtcBuffer[10] = decToBcd(a1hour);
				rtcBuffer[11] = decToBcd(a1day);
				rtcBuffer[12] = decToBcd(a1date);
				rtcBuffer[13] = decToBcd(a2min);
				rtcBuffer[14] = decToBcd(a2hour);
				rtcBuffer[15] = decToBcd(a2day);
				rtcBuffer[16] = decToBcd(a2date);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtcBuffer, 17, 100); HAL_Delay(500);
		}
		
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
		{
			rtcBuffer[0] = 0;	
			HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, rtcBuffer, 1, 100);
			HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcBuffer, 19, 100);
			rtcBuffer[0] = 0; 
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtcBuffer, 1, 100);
			switch (set)
			{
				case 1: if (hour < 23) hour++; else hour = 0; break;
				case 2: if (min < 59) { min++; sec = 0; } else { min = 0; sec = 0; } break;
				case 3: if (day < 7) day++; else day = 1; break;
				case 4: if (date < 31) date++; else date = 1; break;
				case 5: if (month < 12) month++; else month = 1; break;
				case 6: if (year < 99) year++; else year = 0; break;
				case 7: if (a1hour < 23) a1hour++; else a1hour = 0; break;
				case 8: if (a1min < 59) a1min++; else a1min = 0; break;
				case 9: if (a2hour < 23) a2hour++; else a2hour = 0; break;
				case 10: if (a2min < 59) a2min++; else a2min = 0; break;			
			}
				rtcBuffer[1] = decToBcd(sec);
				rtcBuffer[2] = decToBcd(min);
				rtcBuffer[3] = decToBcd(hour);
				rtcBuffer[4] = decToBcd(day);
				rtcBuffer[5] = decToBcd(date);
				rtcBuffer[6] = decToBcd(month);
				rtcBuffer[7] = decToBcd(year);	
				rtcBuffer[8] = decToBcd(a1sec);
				rtcBuffer[9] = decToBcd(a1min);
				rtcBuffer[10] = decToBcd(a1hour);
				rtcBuffer[11] = decToBcd(a1day);
				rtcBuffer[12] = decToBcd(a1date);
				rtcBuffer[13] = decToBcd(a2min);
				rtcBuffer[14] = decToBcd(a2hour);
				rtcBuffer[15] = decToBcd(a2day);
				rtcBuffer[16] = decToBcd(a2date);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, rtcBuffer, 17, 100); HAL_Delay(500);
		}
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
