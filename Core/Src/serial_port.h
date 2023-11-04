
#include <stdio.h>

void uart_send_char(char ch);
void uart_send_string(const char* str);
void uart_send_int(int value);
void uart_send_hex(int value);

void init_serial_port();
