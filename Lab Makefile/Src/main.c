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
#include <stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void Transmit_Char(char c);
void Transmit_String(char str[]);
void configureUART(void);
void configureI2C2(void);
char prompt[6] = {'C', 'M', 'D', '?', '\0'};

//motor globals
void ConfigureMotorOutput(void);
void CheckMoisture(void);
void MotorRoutines(void);
void RoutineA(void);
void RoutineB(void);
void RoutineC(void);
int samples = 10;
int sum_samples;
int avg_samples;
int temp_samples;
int avg_temp;
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
  // enable GPIOB and GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  
  configureUART();
  configureI2C2();
  
  // all LEDS off
  GPIOC->ODR |= (0 << 6)   | (0 << 7)  | (0 << 8)  | (0 << 9);
  
  HAL_Delay(500);
  
  GPIOC->ODR |= (1 << 6);
	
	ConfigureMotorOutput();
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer[sizeof(int) * 4 + 1];
  uint16_t cap, cap_low, cap_high, temp_low, temp_high;
  int num = 0;
	sum_samples = 0;
	int sample_count = 0;
	temp_samples = 0;
	
  while (1)
  {
		if (sample_count >= (samples - 1)) {
			
			avg_samples = sum_samples / sample_count;
			avg_temp = temp_samples / sample_count;
			CheckMoisture();
			sum_samples = 0;
			sample_count = 0;
			temp_samples = 0;
		
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (0x36 << 1); // set slave address to 0x6B
    I2C2->CR2 |= (2 << 16);    // set number of bytes to 2
    I2C2->CR2 &= ~(1 << 10);  // set RD_WRN to write
    I2C2->CR2 |= (1 << 13);    // set the start bit

    // wait for a NACKF or TXIS
    while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

    // write the address of the soil sensor (0x0F)
    I2C2->TXDR = (0x0F << 0);

    // wait for NACKF or TXIS
    while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

    // write the address of the touch function (0x10)
    I2C2->TXDR = (0x10 << 0);

    // wait for TC
    while(!(I2C2->ISR & (1 << 6))) {}
    
    HAL_Delay(1000);

    // read in moisture value
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (0x36 << 1); // set slave address to 0x36
    I2C2->CR2 |= (2 << 16);    // set number of bytes to 2
    I2C2->CR2 |= (1 << 10);    // set RD_WRN to read
    I2C2->CR2 |= (1 << 13);    // set the start bit

    // wait for RXNE or NACKF
    while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

    // read in data
    cap_low = I2C2->RXDR;

    // wait for RXNE or NACKF
    while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

    //cap_high = (I2C2->RXDR << 8); //old
    //cap = cap_high | cap_low;	//old
    //sprintf(buffer, "%d", cap_high/100); //old		
		
		temp_high = (I2C2->RXDR << 8);  //new
		sprintf(buffer, "%d", temp_high/100); //new
    Transmit_String("Temperature: ");
    Transmit_String(buffer);
    Transmit_String("\n");
    Transmit_Char(' ');

    // wait for TC
    while(!(I2C2->ISR & (1 << 6))) {}
    
    HAL_Delay(1000);
    
    // TEMP
    
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (0x36 << 1); // set slave address to 0x6B
    I2C2->CR2 |= (2 << 16);    // set number of bytes to 2
    I2C2->CR2 &= ~(1 << 10);  // set RD_WRN to write
    I2C2->CR2 |= (1 << 13);    // set the start bit

    // wait for a NACKF or TXIS
    while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

    // write the address of the soil sensor (0x0F)
    I2C2->TXDR = (0x00 << 0);

    // wait for NACKF or TXIS
    while(!(I2C2->ISR & ((1 << 4) | (1 << 1)))) {}

    // write the address of the touch function (0x10)
    I2C2->TXDR = (0x04 << 0);

    // wait for TC
    while(!(I2C2->ISR & (1 << 6))) {}
    
    HAL_Delay(1000);

    // read in moisture value
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (0x36 << 1); // set slave address to 0x36
    I2C2->CR2 |= (2 << 16);    // set number of bytes to 2
    I2C2->CR2 |= (1 << 10);    // set RD_WRN to read
    I2C2->CR2 |= (1 << 13);    // set the start bit

    // wait for RXNE or NACKF
    while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

    // read in data
    temp_low = I2C2->RXDR;

    // wait for RXNE or NACKF
    while(!(I2C2->ISR & ((1 << 2) | (1 << 1)))) {}

    //temp_high = (I2C2->RXDR << 8); //Old
    //sprintf(buffer, "%d", temp_high); //Old
		cap_high = (I2C2->RXDR << 8); //New
		sprintf(buffer, "%d", cap_high); //New
    Transmit_String("Capacitance: ");
    Transmit_String(buffer);
    Transmit_Char(' ');

    // wait for TC
    while(!(I2C2->ISR & (1 << 6))) {}

    
//    sprintf(buffer, "%d", num);
//    Transmit_String(buffer);
//    Transmit_Char(' ');
    
    num = num + 1;
		sample_count = sample_count +1;
		sum_samples = sum_samples + cap_high;
		temp_samples = temp_samples + temp_high;
    HAL_Delay(1000);
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

void Transmit_Char(char c) {
  while(!(USART3->ISR & 0x80)) {    // if bit 7 is set
  }
  USART3->TDR = c;
}

void Transmit_String(char str[]) {
  char c = str[0];
  int i = 0;
  while(c != '\0') {
    Transmit_Char(c);
    c = str[++i];
  }
}

void configureUART(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // enable USART
  
  // configure USART
  USART3->BRR = HAL_RCC_GetHCLKFreq()/115200; // set baud rate to 115200 bits/second
  USART3->CR1 |= (1 << 3) | (1 << 2);// enable transmitter and reciever
  USART3->CR1 |= (1 << 0); // enable USART
  USART3->CR1 |= (1 << 5); // enable USART not empty interrupt
  
  // Set PC4 and PC5 to AF mode and AF1
  GPIOC->MODER  |= (1 << 11) | (1 << 9);
  GPIOC->AFR[0] |= (1 << 20) | (1 << 16);
}

void configureI2C2(void) {
  
  GPIOB->MODER |= (1 << 23) | (1 << 27) | (1 << 28); // PB11 AF, PB13 AF, PB14 output
  GPIOC->MODER |= (1 << 0);  // PC0 output
  GPIOC->MODER   |= (1 << 12)   | (1 << 14)
                |  (1 << 16)  | (1 << 18) ; // initialize LEDS
  
  GPIOB->OTYPER |= (1 << 11) | (1 << 13);  // PB11 and PB13 open-drain
  
  GPIOB->PUPDR |= (1 << 22) | (1 << 26);   // PB11 and PB13 to pull-up
  
  GPIOB->AFR[1] |= (1 << 12) | (5 << 20);   // set PB11 to SDA and PB13 to SCL
  
  GPIOB->ODR |= (1 << 14);  // initialize PB14
  GPIOC->ODR |= (1 << 0);    // initialize PC0
  
  /*
   *  initialize I2C2
   */
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;  // emable the ISC2
  
  I2C2->TIMINGR |= (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20) | (1 << 28); // configure timing to 100kHz
  I2C2->CR1 |= (1 << 0); // enable I2C2
}

//Simulates the interrupt from a properly working sensor
void CheckMoisture(void) {
	
	int threshold1 = 28000;
	if (avg_temp > 40) {
		if(avg_samples <= threshold1) {
			MotorRoutines(); //TODO
		}
	}

}

 /*
	* Motor Routines
	*/

//normally the sensor would trigger an interrupt below a certain level and then 
//we'd examine the exact moisture level to determine the routine 
void MotorRoutines(void) {
	
		//higher thresholds mean higher moisture level
	//the lower the current moisture, the longer the motor needs to run
	int threshold1 = 28000;
	int threshold2 = 25000;
	int threshold3 = 20000;
	
	if (avg_samples <= threshold3 ) {
		RoutineC();
		return;
	}
	
	if (avg_samples <= threshold2) {
		RoutineB();
		return;
	}
	
	if (avg_samples <= threshold1) {
		RoutineA();
		return;		
	}
	
	return;

}

// minimal moisture required
void RoutineA(void) {
	//write PC3 high
	GPIOC->ODR |= (0x1 << 3);
	HAL_Delay(5000);
	//write PC3 Low
	GPIOC->ODR &= ~(0x1 << 3); 
}

//medium moisture required
void RoutineB(void) {
	//WRite PC3 High
	GPIOC->ODR |= (0x1 << 3);
	HAL_Delay(10000);
	//Write PC3 Low
	GPIOC->ODR &= ~(0x1 << 3); 
}

//lots of moisture required
void RoutineC(void) {
	//Write PC3 High
	GPIOC->ODR |= (0x1 << 3);
	HAL_Delay(20000);
	//Write PC3 Low
	GPIOC->ODR &= ~(0x1 << 3); 
}

void ConfigureMotorOutput(void) {  
	//Set PC3 to (b01) GeneralPurposeOutputMode
	GPIOC->MODER |= (0x01 << 6);
	GPIOC->OTYPER &= ~(0x1 << 3);
	GPIOC->PUPDR &= ~(0x3 << 6);
	GPIOC->ODR &= ~(0x1 << 3); 

}


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
