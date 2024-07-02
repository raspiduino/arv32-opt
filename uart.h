#ifndef _uart_h
#define _uart_h
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

/************************************
*   uart.h v1 - 11/02/2018
************************************/
#ifdef __AVR__
#define BAUD2BRR(x)     (((F_CPU/((x)*16UL))) - 1)
#define UART_pputs(x)   UART_puts_p(PSTR(x))
#endif

// UART functions
void UART_init(void);
void UART_putc(const unsigned char data);
void UART_puts(const char* charString);
void UART_puthex8(uint8_t val);
unsigned char UART_getc(void);
#ifdef __AVR__
void UART_puts_p(const char* ps);
#endif
unsigned char UART_available(void);
void UART_puthex32(unsigned long value);
void UART_putdec32(unsigned long value);

#endif
