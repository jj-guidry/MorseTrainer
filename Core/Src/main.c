
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "init.h"
#include "serial_port.h"
#include "led_display.h"
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


int main(void)
{

  HAL_Init();
  SystemClock_Config();

  init_serial_port();

  init_timers_gpio();
  init_buzzer_pwm();

  init_display();
  led_startup_commands();

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
