/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
	
	/*
	 *	initialize GPIO 
	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;  // enable GPIOB and GPIOC 
	
	GPIOB->MODER |= (1 << 23) | (1 << 27) | (1 << 28); // PB11 AF, PB13 AF, PB14 output
	GPIOC->MODER |= (1 << 0);	// PC0 output 
	GPIOC->MODER 	|= (1 << 12) 	| (1 << 14) 
								|  (1 << 16)  | (1 << 18) ; // initialize LEDS
	
	GPIOB->OTYPER |= (1 << 11) | (1 << 13);  // PB11 and PB13 open-drain
	
	GPIOB->PUPDR |= (1 << 22) | (1 << 26);   // PB11 and PB13 to pull-up
	
	GPIOB->AFR[1] |= (1 << 12) | (5 << 20);	 // set PB11 to SDA and PB13 to SCL
	
	GPIOB->ODR |= (1 << 14);  // initialize PB14
	GPIOC->ODR |= (1 << 0);		// initialize PC0
	GPIOC->ODR |= (0 << 6)   | (0 << 7)  | (0 << 8)  | (0 << 9)	; // all LEDS off
	
	/*
	 *	initialize I2C2
	 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;  // emable the ISC2
	
	I2C2->TIMINGR |= (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20) | (1 << 28); // configure timing to 100kHz
	I2C2->CR1 |= (1 << 0); // enable I2C2
	
	/* Clear the NBYTES and SADD bit fields
	* The NBYTES field begins at bit 16, the SADD at bit 0
	*/
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
	I2C2->CR2 |= (1 << 16);		// set number of bytes to 1
	I2C2->CR2 |= (1 << 13);		// set the start bit
	

	// wait for a NACKF or TXIS
	while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}
	if(I2C2->ISR & (1 << 4)) {
		// broke
		GPIOC->ODR |= (1 << 6);
	}

	// write the address of the WHO_AM_I register 
	I2C2->TXDR |= (0x0F << 0);
	
	// wait for the TC flag
	while(!(I2C2->ISR & (1 << 6))) {}
	
	// reload CR2 but for read
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
	I2C2->CR2 |= (1 << 16);		// set number of bytes to 1
	I2C2->CR2 |= (1 << 10);		// set RD_WRN to read
	I2C2->CR2 |= (1 << 13);		// set the start bit
		
	// wait for RXNE or NACKF
	while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}
	if(I2C2->ISR & (1 << 4)) {
		// broke
		GPIOC->ODR |= (1 << 6);
	}	
	
	// check for valid response
	if(I2C2->RXDR == 0xD4) {
		GPIOC->ODR ^= (1 << 9);
	}
	else {
		GPIOC->ODR |= (1 << 6);
	}
	
	// set the stop bit
//	I2C2->CR2 |= (1 << 14);
		
	
	/*
	 *	enable the gyroscope
	 */
	
	// write x and y axis to CTRL_REG1 (0x20)
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
	I2C2->CR2 |= (2 << 16);		// set number of bytes to 1
	I2C2->CR2 &= ~(1 << 10);		// set RD_WRN to write
	I2C2->CR2 |= (1 << 13);		// set the start bit
	
	// wait for a NACKF or TXIS
	while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

	// write the address of the CRTL_REG1 register 
	I2C2->TXDR = (0x20 << 0);
		
	// wait for a NACKF or TXIS
	while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}
	
	// write enable X, Y, PD
	I2C2->TXDR = (0xB << 0);
		
	// wait for TC
	while(!(I2C2->ISR & (1 << 6))) {}
		
	// set the stop bit
	I2C2->CR2 |= (1 << 14);
		
	int16_t x, x_low, x_high, y, y_low, y_high;
		GPIOC->ODR &= ~(1 << 9);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(100);
		x = 0;
		y = 0;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		// ask for x values
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
		I2C2->CR2 |= (1 << 16);		// set number of bytes to 1
		I2C2->CR2 &= ~(1 << 10);	// set RD_WRN to write
		I2C2->CR2 |= (1 << 13);		// set the start bit

		// wait for a NACKF or TXIS
		while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

		// write the address of both registers
		I2C2->TXDR = (0xA8 << 0);	
			
		// wait for TC
		while(!(I2C2->ISR & (1 << 6))) {}
			
		// set the stop bit
		I2C2->CR2 |= (1 << 14);
			
		// read x_low val
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
		I2C2->CR2 |= (2 << 16);		// set number of bytes to 2
		I2C2->CR2 |= (1 << 10);		// set RD_WRN to read
		I2C2->CR2 |= (1 << 13);		// set the start bit
			
		// wait for RXNE or NACKF
		while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

		// read in data
		x_low = I2C2->RXDR;
		
			
		// wait for RXNE or NACKF
		while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}
		
		x_high = (I2C2->RXDR << 8);
			
		// wait for TC
		while(!(I2C2->ISR & (1 << 6))) {}
			
		x = x_low | x_high;
			
		if(x > 2000) {
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		else if(x < -2000){
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR |= (1 << 9);
		}
		else {
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
	
		
		/*
		*		Y VALUES
		*/
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
		I2C2->CR2 |= (1 << 16);		// set number of bytes to 1
		I2C2->CR2 &= ~(1 << 10);	// set RD_WRN to write
		I2C2->CR2 |= (1 << 13);		// set the start bit

		// wait for a NACKF or TXIS
		while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}
					GPIOC->ODR |= (1 << 7);
		// write the address of both registers
		I2C2->TXDR = (0xAA << 0);	
			
		// wait for TC
		while(!(I2C2->ISR & (1 << 6))) {}
			
		// read x_low val
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x6B << 1); // set slave address to 0x6B
		I2C2->CR2 |= (2 << 16);		// set number of bytes to 2
		I2C2->CR2 |= (1 << 10);		// set RD_WRN to read
		I2C2->CR2 |= (1 << 13);		// set the start bit
			
		// wait for RXNE or NACKF
		while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

		// read in data
		y_low = I2C2->RXDR;
			
		// wait for RXNE or NACKF
		while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}
		
		y_high = (I2C2->RXDR << 8);
			
		// wait for TC
		while(!(I2C2->ISR & (1 << 6))) {}

		// set the stop bit
		I2C2->CR2 |= (1 << 14);
			
		y = y_low | y_high;
			
		if(y > 2000) {
			GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~(1 << 6);
		}
		else if(y < -2000){
			GPIOC->ODR &= ~(1 << 7);
			GPIOC->ODR |= (1 << 6
			);
		}
		else {
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
