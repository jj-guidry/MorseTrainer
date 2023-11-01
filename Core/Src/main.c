
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

// for the timer clocks to be initialized, we force an update event so that
// the PSC's are loaded into their respective shadow registers
volatile uint8_t forcedEventFlag = 0;
// bit 0 is for tim6
// bit 1 is for tim2

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
    uart_send_char(' ');
}


// tim3 used for PWM for buzzer
void init_tim3(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // en tim3 clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // en GPIOC clock
	GPIOC->MODER |= GPIO_MODER_MODER6_1; // led on PC6 to AF
	TIM3->PSC = 480-1;
	TIM3->ARR = 156-1; // 1hz pwm signal
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;  // interrupt when CNT == CCR1
	TIM3->CCR1 = 16; // pwm goes low @ cnt == 500
	TIM3->CNT = 17; // start pwm thing low
	TIM3->CCER |= TIM_CCER_CC1E;
}

void init_uart(){

	// ------------------ serial port UART setup ----------------

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN; // enable GPIOD and GPIOC clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
	// set MODER's to alternate function
	GPIOC->MODER |= GPIO_MODER_MODER12_1;
	GPIOC->MODER &= ~(GPIO_MODER_MODER12_0);

	GPIOD->MODER |= GPIO_MODER_MODER2_1;
	GPIOD->MODER &= ~GPIO_MODER_MODER2_0;

	// configure pc12 for UART5_TX (AF2)
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12;
	GPIOC->AFR[1] |= (2 << 16);

	// configure pd2 for UART5_RX (AF2)
	GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2;
	GPIOD->AFR[0] |= (2 << 8);

	// first, turn off the UE bit
	USART5->CR1 &= ~USART_CR1_UE;

	// set word length to 8 bits
	USART5->CR1 &= ~USART_CR1_M0;
	USART5->CR1 &= ~USART_CR1_M1;

	// set for 1 stop bit
	USART5->CR2 &= ~(USART_CR2_STOP);

	// no parity
	USART5->CR1 &= ~(USART_CR1_PCE);

	// 16x over-sampling
	USART5->CR1 &= ~(USART_CR1_OVER8);

	// baud rate 115200
	USART5->BRR = 0x1a1;

	// enable receiver and transmitter
	USART5->CR1 |= (USART_CR1_TE | USART_CR1_RE);

	// enable UART
	USART5->CR1 |= USART_CR1_UE;

	// wait for things to work?
	while(!(USART5->ISR & USART_ISR_REACK) || !(USART5->ISR & USART_ISR_TEACK));

}



void EXTI0_1_IRQHandler(){

	// ack interrupt -> clear 1st bit of EXTI->PR
	EXTI->PR = 1<<1;

	// toggle pc7 led
	if((GPIOA->IDR >> 1) & 1){ // button pin is high (triggered by rising edge)

		// turn on led
		GPIOC->BSRR = (1 << 7);

		// turn on buzzer
		TIM3->CR1 |= TIM_CR1_CEN;

		// stop the inactivity timer
		TIM2->CR1 &= ~1;
		TIM2->CNT = 0;

		// start activity timer
		TIM6->CR1 |= 1;

	} else { // button pin is low (triggered by falling edge)

		 // turn off the led
		GPIOC->BSRR = (1 << 7) << 16;

		// turn off buzzer
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->CNT = 17; // cnt > ccr1 to make sure line is low

		int count = (int) TIM6->CNT;

		// stop activity timer since button went low
		TIM6->CR1 &= ~1;
		TIM6->CNT = 0;

		// determine if short or long via CNT data
		if(count < 115){ // press was shorter than 115ms -> dit
			// received a dit, update state
			state = morse_tree[(int)state][0];
		} else {
			// received a dash, update state
			state = morse_tree[(int)state][1];
		}

		// start inactivity timer
		TIM2->CR1 |= TIM_CR1_CEN;

	}
}

void TIM6_DAC_IRQHandler(){
	if(!(forcedEventFlag & 1)){ // if this is the first interrupt
		forcedEventFlag |= 1; // set the 0th bit to ack init event
		TIM6->SR &= ~TIM_SR_UIF;
	} else {
		// otherwise, interrupt triggered by holding down for longer
		// than 350ms, indicating a dash, but we only should turn off
		// timer and set its cnt to 399
		TIM6->SR &= ~TIM_SR_UIF;
		TIM6->CR1 &= ~TIM_CR1_CEN; // stop the timer
		TIM6->CNT = 399; // max out the timer so that a dash is interpreted
	}
}

void TIM2_IRQHandler(){

	// check if interrupt triggered from 3 inactive time units (end of curr dit/dash)
	// or from 7 inactive time units (end of word, send a space)

	// bit 1 of forcedEventFlag == 0 means this is the first interrupt (wrong clock speed)
	if(!(forcedEventFlag & 1<<1)){
		forcedEventFlag |= 1<<1; // set that bit to ack this
		TIM2->SR &= ~TIM_SR_UIF;
		return;
	}

	if(TIM2->SR & TIM_SR_CC1IF){
		// interrupt source: 3 inactive time units
		TIM2->SR &= ~(TIM_SR_CC1IF); // clear CC1IF bit
		// and send the current char to serial port
		uart_send_char(node_decode[(int)state]);

	} else if(TIM2->SR & TIM_SR_UIF){
		// interrupt source: 7 inactive time units
		TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
		TIM2->CR1 &= ~TIM_CR1_CEN; // stop the timer
		// send a space, since this means end of current word
		uart_send_char(' ');
		TIM2->CNT = 0;
	}

	// restart state
	state = START;

}

// PA0 for input
void init_timers_gpio(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // gpioa en clock for input button
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // gpioc en clock for output led
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // syscfg en clock

	GPIOA->MODER &= ~(GPIO_MODER_MODER1); // pa1 for input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1; // pa1 pull down resistor

	// pc7 as output
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_1);


	// config interrupt on pa0
	SYSCFG->EXTICR[0] &= ~(0xf<<4); // clear bottom 4 bits to set interrupt on pa1
	// call ISR for both rising and falling edge
	EXTI->RTSR |= EXTI_RTSR_RT1;
	EXTI->FTSR |= EXTI_FTSR_FT1;
	// unmask the interrupt on pin 1
	EXTI->IMR |= EXTI_IMR_IM1;

    NVIC->ISER[0] = 1<<EXTI0_1_IRQn;

    // setup timers
    // TIM6 counts duration of press
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // en clock for tim6
    TIM6->PSC = 48000-1;
    TIM6->ARR = 400-1;

    TIM6->EGR |= TIM_EGR_UG; // force an update to init the PSC shadow register

    // TIM2 counts duration between presses
    // 2 interrupts: one at 3 time units (cnt == 3000) to end the character and another at 7 to add a space and pause the whole thing
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 48000-1;
    TIM2->ARR = 700-1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->DIER |= TIM_DIER_CC1IE;
    TIM2->CCR1 = 300;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
    TIM2->CNT = 0;

    TIM2->EGR |= TIM_EGR_UG; // force an update

    NVIC->ISER[0] = 1<<TIM2_IRQn;

}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  init_uart();
  init_timers_gpio();
  init_tim3();
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
