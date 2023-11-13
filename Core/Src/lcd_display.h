
#include <stdio.h>
#include <stdint.h>

void lcd_send_char(char ch, uint8_t* cursor_pos);
void lcd_startup_commands();
void init_display();
void lcd_send_command(uint16_t cmd);
