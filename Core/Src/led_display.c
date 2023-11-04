

#include <stdio.h>
#include "led_display.h"
#include "main.h"


// pa15 = nCS
// pa5 = clk
// pa7 = d1


void led_startup_commands(){
	HAL_Delay(1);
	led_send_command(0x38);
	led_send_command(0x08);
	led_send_command(0x01);

	HAL_Delay(2);
	led_send_command(0x06);
	led_send_command(0x02);
	led_send_command(0x0c);

}
void init_display(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER7_1; // pa7 as AF
	GPIOA->MODER |= GPIO_MODER_MODER5_1; // pa5 as AF
	GPIOA->MODER |= GPIO_MODER_MODER15_1; // pa15 as AF

	GPIOA->AFR[0] &= ~(0xf << 28); // pa7 as AF0 -> CLK
	GPIOA->AFR[0] &= ~(0xf << 20); // pa5 as AF0  -> MOSI
	GPIOA->AFR[1] &= ~(0xf << 28); // pa15 as AF0 -> nSS

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 &= ~SPI_CR1_SPE; // turn off channel before config
	while(SPI1->CR1 & SPI_CR1_SPE); // wait for it to be off
	SPI1->CR1 |= SPI_CR1_BR; // set baud rate to all 1's
	SPI1->CR2 |= SPI_CR2_DS_3 | SPI_CR2_DS_0; // 1001 DS = 10 bit word
	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR2 |= SPI_CR2_NSSP;
	SPI1->CR1 |= SPI_CR1_SPE;

}


void led_send_command(uint16_t cmd){
	while(!(SPI1->SR & SPI_SR_TXE)); // wait for there to be a transmission
	SPI1->DR = cmd;
}


void led_send_char(char ch){
	led_send_command(0x02);
	led_send_command(0x200 | ch);
}
