/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <Lcd.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lcd.h"
#include "Pcf8574t.h"
#include "Lcdgpio.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
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

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */


	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	Lcd_gpio::GPIO_control_union gpios;
	Lcd_gpio::PORT_control_union ports;

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIODEN;

	for (int i = 0; i < 8; ++i) {
		GPIOD->MODER |= (1 << (i*2));
		GPIOD->MODER &= ~(10 << (i*2));

		GPIOD->PUPDR |= (1 << (i*2));
		GPIOD->PUPDR &= ~(10 << (i*2));
	}
	for (int i = 3; i < 6; ++i) {
		GPIOB->MODER |= (1 << (i*2));
		GPIOB->MODER &= ~(10 << (i*2));

		GPIOB->PUPDR |= (1 << (i*2));
		GPIOB->PUPDR &= ~(10 << (i*2));
	}

	gpios.gpios.GPIO_RS = GPIOB;
	gpios.gpios.GPIO_RW = GPIOB;
	gpios.gpios.GPIO_E = GPIOB;
#ifndef LCD_4_BIT
	gpios.gpios.GPIO_D0 = GPIOD;
	gpios.gpios.GPIO_D1 = GPIOD;
	gpios.gpios.GPIO_D2 = GPIOD;
	gpios.gpios.GPIO_D3 = GPIOD;
#endif
	gpios.gpios.GPIO_D4 = GPIOD;
	gpios.gpios.GPIO_D5 = GPIOD;
	gpios.gpios.GPIO_D6 = GPIOD;
	gpios.gpios.GPIO_D7 = GPIOD;
	gpios.gpios.GPIO_LED = GPIOB;

	ports.ports.PORT_RS = 5;
	ports.ports.PORT_RW = 4;
	ports.ports.PORT_E = 3;
#ifndef LCD_4_BIT
	ports.ports.PORT_D0 = 7;
	ports.ports.PORT_D1 = 6;
	ports.ports.PORT_D2 = 5;
	ports.ports.PORT_D3 = 4;
#endif
	ports.ports.PORT_D4 = 3;
	ports.ports.PORT_D5 = 2;
	ports.ports.PORT_D6 = 1;
	ports.ports.PORT_D7 = 0;
	ports.ports.PORT_LED = 8;

	Lcd_gpio::Lcd_gpio lcd_gpio = Lcd_gpio::Lcd_gpio(&gpios, &ports);
//	lcd_gpio.send_half_byte(0b1111);
//	pcf::Pcf8574t pcf = pcf::Pcf8574t(0b01001111, &example_transmit_to_lcd_i2c,
//						      &example_recive_from_lcd_i2c);

//	lcd::Lcd lcd_i2c = lcd::Lcd(&pcf);
	lcd::Lcd lcd_pins = lcd::Lcd(&lcd_gpio);
//	lcd_pins.init();
	uint8_t ch = 0;
	lcd_pins.write_string((uint8_t *)"Hi world!!");
	LL_mDelay(1000);
	while (1)
	{
		lcd_pins.enable_cursor(false);
		LL_mDelay(2000);
		lcd_pins.enable_cursor(true);
		LL_mDelay(2000);
		lcd_pins.enable_blink(false);
		LL_mDelay(2000);
		lcd_pins.enable_blink(true);
		LL_mDelay(2000);
		lcd_pins.enable_cursor(false);
		LL_mDelay(2000);
		lcd_pins.enable_cursor(true);
		LL_mDelay(2000);
		lcd_pins.enable_display(false);
		LL_mDelay(2000);
		lcd_pins.enable_display(true);
		LL_mDelay(2000);
		for (int var = 0; var < 16; ++var) {
			for (int var2 = 0; var2 < 4; ++var2) {
				lcd_pins.cursor_set_pos(var, var2);
				lcd_pins.set_display_shift(lcd::SHIFT_RIGHT, 1);
				lcd_pins.write_symbol(ch++);
				LL_mDelay(200);
			}
		}
//		lcd_i2c.write_string((uint8_t *)("Hellow!!!"));
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
		Error_Handler();
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while(LL_RCC_HSI_IsReady() != 1)
	{

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while(LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	LL_Init1msTick(64000000);
	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
	LL_SetSystemCoreClock(64000000);
	LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_HSI);
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	LL_I2C_InitTypeDef I2C_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	/**I2C2 GPIO Configuration
  PF0-OSC_IN   ------> I2C2_SDA
  PF1-OSC_OUT   ------> I2C2_SCL
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

	/* I2C2 interrupt Init */
	NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(I2C2_ER_IRQn);

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	/**I2C Initialization
	 */
	LL_I2C_EnableAutoEndMode(I2C2);
	LL_I2C_DisableOwnAddress2(I2C2);
	LL_I2C_DisableGeneralCall(I2C2);
	LL_I2C_EnableClockStretching(I2C2);
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x2000090E;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C2, &I2C_InitStruct);
	LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

}

/* USER CODE BEGIN 4 */
void example_transmit_to_lcd_i2c(const uint8_t addres, const uint8_t data){
	LL_I2C_HandleTransfer(I2C2, addres, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	/* Loop until STOP flag is raised  */
	while(!LL_I2C_IsActiveFlag_STOP(I2C2))
	{
		/* (2.1) Transmit data (TXIS flag raised) *********************************/

		/* Check TXIS flag value in ISR register */
		if(LL_I2C_IsActiveFlag_TXIS(I2C2))
		{
			/* Write data in Transmit Data register.
	      TXIS flag is cleared by writing data in TXDR register */
			LL_I2C_TransmitData8(I2C2, data);
		}
	}

	/* End of I2C_SlaveReceiver_MasterTransmitter Process */
	LL_I2C_ClearFlag_STOP(I2C2);


//	LL_I2C_HandleTransfer(I2C2, addres, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
//
//	while(!LL_I2C_IsActiveFlag_STOP(I2C2)){
//		LL_I2C_TransmitData8(I2C2, data);
//	}
//	LL_I2C_ClearFlag_STOP(I2C2);
}

// addres - 0b01001111
uint8_t example_recive_from_lcd_i2c(const uint8_t addres){
	uint8_t ans_recv = 0;
	LL_I2C_HandleTransfer(I2C2, addres, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	while(!LL_I2C_IsActiveFlag_STOP(I2C2)) {
		/* Receive data (RXNE flag raised) */

		/* Check RXNE flag value in ISR register */
		if(LL_I2C_IsActiveFlag_RXNE(I2C2))
		{
			/* Read character in Receive Data register.
	      RXNE flag is cleared by reading data in RXDR register */
			ans_recv = LL_I2C_ReceiveData8(I2C2);
		}
	}
	LL_I2C_ClearFlag_STOP(I2C2);
	return ans_recv;
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

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
