
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void TIM6_DAC_IRQHandler(){
	TIM6->SR &= ~1;

	if((GPIOC->IDR >> 7) & 1){
		GPIOC->BSRR = (1 << 7) << 16;
	} else {
		GPIOC->BSRR = (1 << 7);
	}

}

void send_char_over_uart(char ch){
	while (!(USART5->ISR & (1 << 7)));
	USART5->TDR = ch;
}

void init_uart(){
	RCC->AHBENR |= 3 << 19; // enable GPIOB and GPIOC clocks
	RCC->APB1ENR |= 1 << 20;
	// set moders to alternate function
	GPIOC->MODER |= 1 << 25;
	GPIOC->MODER &= ~(1 << 24);

	GPIOD->MODER |= (1<<5);
	GPIOD->MODER &= ~(1<<4);

	// configure pc12 for UART5_TX (AF2)
	GPIOC->AFR[1] |= 2 << 16;
	GPIOC->AFR[1] &= ~(0xd << 16);

	// configure pd2 for UART5_RX (AF2)
	GPIOD->AFR[0] |= 2 << 8;
	GPIOD->AFR[0] &= ~(0xd << 8);

	// first, turn off the UE bit
	USART5->CR1 &= ~1;

	// set word length to 8 bits
	USART5->CR1 &= ~(1<<28);
	USART5->CR1 &= ~(1<<12);

	// set for 1 stop bit
	USART5->CR2 &= ~(3<<12);

	// no parity
	USART5->CR1 &= ~(1<<10);

	// 16x oversampling
	USART5->CR1 &= ~(1<<15);

	// baud rate 115200
	USART5->BRR = 0x1a1;

	// enable receiver and transmitter
	USART5->CR1 |= 3 << 2;

	// enable uart
	USART5->CR1 |= 1;

	// wait for things to work?
	while(!(USART5->ISR & (1<<22)) || !(USART5->ISR & (1<<21)));

}

void init_timer6(){
	RCC->AHBENR |= 1<<19; // gpioc en clock
	GPIOC->MODER |= 1 << 14; // pin7 for output

	RCC->APB1ENR |= 1<<4; // tim6 en clock
	TIM6->PSC = 4800-1;
	TIM6->ARR = 10000-1;
	TIM6->DIER |= 1;
	TIM6->CR1 |= 1;

	NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;


}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  init_uart();
  init_timer6();

  while (1)
  {

	  send_char_over_uart('J');
	  HAL_Delay(1000);
  }

}

// set system clock to HSI with PLL for 48MHz clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  // Set the PLL source to HSI
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;  // Multiply by 6 to get 48 MHz (8 MHz * 6 = 48 MHz)
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;  // Set the pre-divider to 1
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}




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

#ifdef  USE_FULL_ASSERT
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
