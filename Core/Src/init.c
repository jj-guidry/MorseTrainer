
#include <stdio.h>
#include "init.h"
#include "main.h"

// tim3 used for PWM for buzzer
void init_buzzer_pwm(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // en tim3 clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // en GPIOC clock
	GPIOC->MODER |= GPIO_MODER_MODER6_1; // led on PC6 to AF
	TIM3->PSC = 480-1;
	TIM3->ARR = 156-1; // 1hz pwm signal
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // interrupt when CNT == CCR1
	TIM3->CCR1 = 8; // pwm goes low @ cnt == 8
	TIM3->CNT = 9; // start pwm line low
	TIM3->CCER |= TIM_CCER_CC1E; // enable channel 1
}



// input button and associated timers
void init_timers_gpio(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // gpioa en clock for input button
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // gpioc en clock for output led
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // syscfg en clock

	GPIOA->MODER &= ~(GPIO_MODER_MODER1); // pa1 for input
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1; // pa1 pull down resistor

	// pc7 as output
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOC->MODER &= ~(GPIO_MODER_MODER7_1);


	// config interrupt on pa1
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

void init_mode_switch(){

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIOB->MODER &= ~GPIO_MODER_MODER7; // PB7 as input
	GPIOB->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0; // pb5 and pb6 as output


	SYSCFG->EXTICR[1] &= ~(0xf<<12); // clear pin 7
	SYSCFG->EXTICR[1] |= (1<<12); // set lowest pin for pin 7 routed to port b

	// interrupt on both rising and falling edges
	EXTI->RTSR |= EXTI_RTSR_RT7;
	EXTI->FTSR |= EXTI_FTSR_FT7;

	// unmask interrupt
	EXTI->IMR |= EXTI_IMR_IM7;

	NVIC->ISER[0] = 1 << EXTI4_15_IRQn;

}

void set_tx_mode(){
	// set green (pb6)
	// clear red (pb5)
	// clear screen
	// config RF to send chars recieved
	// enable button interrupt

	// enable the input button
	EXTI->IMR |= EXTI_IMR_IM1;


	//------------ rf stuff start -----------


	//------------  rf stuff stop ------------

	// clear the screen
	lcd_send_command(0x1); // clear screen
	wait(2000000); // wait 2ms (as per documentation)
	lcd_send_command(0x2); // return home / set DDRAM address to 0x00



	// toggle LEDS
	GPIOB->BSRR = 1<<6;
	GPIOB->BRR = 1<<5;
}

void set_rx_mode(){
	// set red (pb5)
	// clear green (pb6)
	// clear screen
	// config rf to listen for chars
	// disable button interrupt

	// disable the button
	EXTI->IMR &= ~EXTI_IMR_IM1;

	//------------ rf stuff start -----------


	//------------  rf stuff stop ------------

	// clear the screen
	lcd_send_command(0x1); // clear screen
	wait(2000000); // wait 2ms (as per documentation)
	lcd_send_command(0x2); // return home / set DDRAM address to 0x00

	// toggle LEDS
	GPIOB->BSRR = 1<<5;
	GPIOB->BRR = 1<<6;
}

