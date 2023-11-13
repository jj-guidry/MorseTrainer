

#include "lcd_display.h"

#include <stdio.h>
#include "main.h"


// pa15 = nCS
// pa5 = clk
// pa7 = d1

// wait n nanoseconds
void wait(long n){
  for(long i = n; i >= 0; i-=83);
}

void lcd_startup_commands(){
	HAL_Delay(1);
	lcd_send_command(0x38); // function set: data length to 8 and font to english/japanese
	lcd_send_command(0x08); //
	lcd_send_command(0x01);

	HAL_Delay(2);
	lcd_send_command(0x06);
	lcd_send_command(0x02);
	lcd_send_command(0x0f);

}
void init_display(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER7_1; // pa7 as AF
	GPIOA->MODER |= GPIO_MODER_MODER5_1; // pa5 as AF
	GPIOA->MODER |= GPIO_MODER_MODER15_1; // pa15 as AF

	GPIOA->AFR[0] &= ~(0xf << 28); // pa7 as AF0 -> MOSI
	GPIOA->AFR[0] &= ~(0xf << 20); // pa5 as AF0  -> CLK
	GPIOA->AFR[1] &= ~(0xf << 28); // pa15 as AF0 -> nSS

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 &= ~SPI_CR1_SPE; // turn off channel before config
	while(SPI1->CR1 & SPI_CR1_SPE); // wait for it to be off
	SPI1->CR1 |= SPI_CR1_BR; // set baud rate to all 1's
	SPI1->CR2 |= SPI_CR2_DS_3; // setting the DS is really wierd, see documentation, will be 0xf after this line
	SPI1->CR2 &= ~(SPI_CR2_DS_2 | SPI_CR2_DS_1); // clear middle two bits for 1001 -> 10 bit word size
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR2 |= SPI_CR2_NSSP;
	SPI1->CR1 |= SPI_CR1_SPE;

}


void lcd_send_command(uint16_t cmd){
	while(!(SPI1->SR & SPI_SR_TXE)); // wait tx buffer empty
	SPI1->DR = cmd;
}


void lcd_send_char(char ch, uint8_t* cursor_pos){
	// adjust the display if needed (wrapping to second line or clearing screen to start over)
	if(*cursor_pos == 16){
		lcd_send_command(0xc0); // go to second line (set DDRAM address to 0x40)
	} else if(*cursor_pos == 32){
		lcd_send_command(0x1); // clear screen
		wait(2000000); // wait 2ms (as per documentation)
		lcd_send_command(0x2); // return home / set DDRAM address to 0x00
		*cursor_pos = -1; // anticipating that it will incremented to 0 at the end of this functions
	}

	lcd_send_command(0x200 | ch); // send the char

	*cursor_pos += 1;

}
