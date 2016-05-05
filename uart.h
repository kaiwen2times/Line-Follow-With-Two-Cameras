#ifndef UART_H_
#define UART_H_


#include <stdint.h>

void put(char *ptr_str);
void putnumU(int i);
void uart_init(void);
void uart_putchar(char ch);
uint8_t uart_getchar(void);
void uart_put(char *ptr_str);

#endif
