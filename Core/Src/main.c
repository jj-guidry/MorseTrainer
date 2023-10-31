
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

enum {
	START = 0,
	E,
	T,
	I,
	A,
	N,
	M,
	S,
	U,
	R,
	W,
	D,
	K,
	G,
	O,
	H,
	V,
	F,
	T1,
	L,
	T2,
	P,
	J,
	B,
	X,
	C,
	Y,
	Z,
	Q,
	T3,
	T4,
	FIVE,
	FOUR,
	THREE,
	TWO,
	PLUS,
	ONE,
	SIX,
	EQUALS,
	SLASH,
	SEVEN,
	EIGHT,
	NINE,
	ZERO,
	ERROR_STATE,
	TREE_SIZE
};


const char morse_tree[TREE_SIZE][2] = {
    [START] = {E, T},
    [E] = {I, A},
    [T] = {N, M},
    [I] = {S, U},
    [A] = {R, W},
    [N] = {D, K},
    [M] = {G, O},
    [S] = {H, V},
    [U] = {F, T1},
    [R] = {L, T2},
    [W] = {P, J},
    [D] = {B, X},
    [K] = {C, Y},
    [G] = {Z, Q},
    [O] = {T3, T4},
    [H] = {FIVE, FOUR},
    [V] = {ERROR_STATE, THREE},
    [F] = {ERROR_STATE, ERROR_STATE},
	[T1] = {ERROR_STATE, TWO},
    [L] = {ERROR_STATE, ERROR_STATE},
	[T2] = {PLUS, ERROR_STATE},
    [P] = {ERROR_STATE, ERROR_STATE},
    [J] = {ERROR_STATE, ONE},
    [B] = {SIX, EQUALS},
    [X] = {SLASH, ERROR_STATE},
    [C] = {ERROR_STATE, ERROR_STATE},
    [Y] = {ERROR_STATE, ERROR_STATE},
    [Z] = {SEVEN, ERROR_STATE},
    [Q] = {ERROR_STATE, ERROR_STATE},
	[T3] = {EIGHT, ERROR_STATE},
	[T4] = {NINE, ZERO},
    [FIVE] = {ERROR_STATE, ERROR_STATE},
    [FOUR] = {ERROR_STATE, ERROR_STATE},
    [THREE] = {ERROR_STATE, ERROR_STATE},
    [TWO] = {ERROR_STATE, ERROR_STATE},
    [PLUS] = {ERROR_STATE, ERROR_STATE},
    [ONE] = {ERROR_STATE, ERROR_STATE},
    [SIX] = {ERROR_STATE, ERROR_STATE},
    [EQUALS] = {ERROR_STATE, ERROR_STATE},
    [SLASH] = {ERROR_STATE, ERROR_STATE},
    [SEVEN] = {ERROR_STATE, ERROR_STATE},
    [EIGHT] = {ERROR_STATE, ERROR_STATE},
    [NINE] = {ERROR_STATE, ERROR_STATE},
    [ZERO] = {ERROR_STATE, ERROR_STATE},
    [ERROR_STATE] = {ERROR_STATE, ERROR_STATE}  // Error state
};

const char node_decode[TREE_SIZE] = {
	[START] = '>',
	[E] = 'E',
	[T] = 'T',
	[I] = 'I',
	[A] = 'A',
	[N] = 'N',
	[M] = 'M',
	[S] = 'S',
	[U] = 'U',
	[R] = 'R',
	[W] = 'W',
	[D] = 'D',
	[K] = 'K',
	[G] = 'G',
	[O] = 'O',
	[H] = 'H',
	[V] = 'V',
	[F] = 'F',
	[T1] = '?',
	[L] = 'L',
	[T2] = '?',
	[P] = 'P',
	[J] = 'J',
	[B] = 'B',
	[X] = 'X',
	[C] = 'C',
	[Y] = 'Y',
	[Z] = 'Z',
	[Q] = 'Q',
	[T3] = '?',
	[T4] = '?',
	[FIVE] = '5',
	[FOUR] = '4',
	[THREE] = '3',
	[TWO] = '2',
	[PLUS] = '+',
	[ONE] = '1',
	[SIX] = '6',
	[EQUALS] = '=',
	[SLASH] = '/',
	[SEVEN] = '7',
	[EIGHT] = '8',
	[NINE] = '9',
	[ZERO] = '0',
	[ERROR_STATE] = '?'  // Error state
};

char state = START; // init to start state

void SystemClock_Config(void);

void uart_send_char(char ch){
	while (!(USART5->ISR & (1 << 7)));
	USART5->TDR = ch;
}

void uart_send_string(const char* str) {
    while (*str) {  // Continue until the null terminator is encountered
        uart_send_char(*str);
        str++;  // Move to the next character in the string
    }
}

void uart_send_int(int value) {
    char buffer[12];  // Buffer to hold the string representation of the integer
    snprintf(buffer, sizeof(buffer), "%d", value);  // Convert the integer to a string
    uart_send_string(buffer);  // Send the string over UART
}

void uart_send_hex(int value) {
    char buffer[12];  // Buffer to hold the string representation of the integer
    snprintf(buffer, sizeof(buffer), "%x", value);  // Convert the integer to a string
    uart_send_string(buffer);  // Send the string over UART
}

void init_uart(){
	RCC->AHBENR |= 3 << 19; // enable GPIOB and GPIOC clocks
	RCC->APB1ENR |= 1 << 20;
	// set MODER's to alternate function
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

	// 16x over-sampling
	USART5->CR1 &= ~(1<<15);

	// baud rate 115200
	USART5->BRR = 0x1a1;

	// enable receiver and transmitter
	USART5->CR1 |= 3 << 2;

	// enable UART
	USART5->CR1 |= 1;

	// wait for things to work?
	while(!(USART5->ISR & (1<<22)) || !(USART5->ISR & (1<<21)));

}


void EXTI0_1_IRQHandler(){

	// ack interrupt -> clear 1st bit of EXTI->PR
	EXTI->PR = 1<<1;

	// toggle pc7 led
	if((GPIOA->IDR >> 1) & 1){ // button pin is high (triggered by rising edge)

		GPIOC->BSRR = (1 << 7); // turn on led

		// stop the inactivity timer
		TIM2->CR1 &= ~1;
		TIM2->CNT = 0;

		// start activity timer
		TIM6->CR1 |= 1;

	} else { // button pin is low (triggered by falling edge)
		GPIOC->BSRR = (1 << 7) << 16; // turn off the led

		int count = (int) TIM6->CNT;

		// stop activity timer since button went low

		TIM6->CR1 &= ~1;
		TIM6->CNT = 0;
		// determine if short or long via CNT data
		if(count < 1150){
			// received a dit, update state
			state = morse_tree[(int)state][0];
		} else {
			// received a dash, update state
			state = morse_tree[(int)state][1];
		}

		// start inactivity timer
		TIM2->CR1 |= 1;

	}
}

void TIM2_IRQHandler(){

	// check if interrupt triggered from 3 inactive time units (end of curr dit/dash)
	// or from 7 inactive time units (end of word, send a space)

	uart_send_hex(TIM2->SR);
	if((TIM2->SR >> 1) & 1){
		// interrupt source: 3 inactive time units
		TIM2->SR &= ~(1<<1); // clear CC1IF bit
		// and send the current char to serial port
		uart_send_char(node_decode[(int)state]);
	} else if(TIM2->SR & 1){
		// interrupt source: 7 inactive time units
		TIM2->SR &= ~1; // clear UIF flag
		TIM2->CR1 &= ~1; // stop the timer
		// send a space, since this means end of current work
		uart_send_char(' ');
		TIM2->CNT = 0;
	}

	// restart state
	state = START;

}

// PA0 for input
void init_timers_gpio(){
	RCC->AHBENR |= 1<<17; // gpioa en clock for input button
	RCC->AHBENR |= 1<<19; // gpioc en clock for output led
	RCC->APB2ENR |= 1; // syscfg en clock

	GPIOA->MODER &= ~(3<<2); // pa1 for input
	GPIOA->PUPDR |= 1<<3; // pa1 pull down resistor

	// pc7 as output
	GPIOC->MODER |= 1 << 14;
	GPIOC->MODER &= ~(1 << 15);


	// config interrupt on pa0
	SYSCFG->EXTICR[0] &= ~(0xf<<4); // clear bottom 4 bits to set interrupt on pa1
	// call ISR for both rising and falling edge
	EXTI->RTSR |= 1<<1;
	EXTI->FTSR |= 1<<1;
	// unmask the interrupt on pin 1
	EXTI->IMR |= 1<<1;

    NVIC->ISER[0] = 1<<EXTI0_1_IRQn;

    // setup timers
    // TIM6 counts duration of press
    RCC->APB1ENR |= 1<<4; // en clock for tim6
    TIM6->PSC = 4800-1;
    TIM6->ARR = 65534-1;
    // TIM2 counts duration between presses
    // 2 interrupts: one at 3 time units (cnt == 3000) to end the character and another at 7 to add a space and pause the whole thing
    RCC->APB1ENR |= 1<<0;
    TIM2->PSC = 4800-1;
    TIM2->ARR = 7000-1;
    TIM2->DIER |= 1;
    TIM2->DIER |= 1<<1;
    TIM2->CCR1 = 3000;
    TIM2->CCMR1 |= (3<<4);
    TIM2->CNT = 0;
    NVIC->ISER[0] = 1<<TIM2_IRQn;

}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  init_uart();
  init_timers_gpio();

  uart_send_string("\n\r");
  for(;;);

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
