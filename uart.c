#include <avr/io.h>
#include <avr/pgmspace.h>
#include "uart.h"

const char hex_digits[] = "0123456789ABCDEF";

void UART_init()
{
    #ifndef BAUD_RATE
    #define BAUD_RATE 9600
    #endif

	// set rate
	UBRR0H = (unsigned char) (((F_CPU/(BAUD_RATE*16UL))) - 1) >> 8;
	UBRR0L = (unsigned char) ((F_CPU/(BAUD_RATE*16UL))) - 1;

	// Enable reciever and transmitter
	UCSR0B |= (1 << RXEN0)|(1 << TXEN0);
}

void UART_putc(const unsigned char data)
{
	// wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));

	// send data to output register
	UDR0 = data;
}

void UART_puts(const char* charString)
{
	// iterate through string
	while(*charString > 0)
		// print character
		UART_putc(*charString++);
}

void UART_puthex8(uint8_t val)
{
    // extract upper and lower nibbles from input value
    uint8_t upperNibble = (val & 0xF0) >> 4;
    uint8_t lowerNibble = val & 0x0F;

    // convert nibble to its ASCII hex equivalent
    upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
    lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

    // print the characters
    UART_putc(upperNibble);
    UART_putc(lowerNibble);
}

unsigned char UART_getc(void)
{
	// wait for data to be received
	while(!(UCSR0A & (1 << RXC0)));

	// get data to output register
	return UDR0;
}

void UART_puts_p(const char* ps)
{
    register char c;

    while ((c = pgm_read_byte(ps++)))
        UART_putc(c);
}

unsigned char UART_available(void)
{
	return UCSR0A & (1 << RXC0);
}

void UART_puthex32(unsigned long value) {
    UART_pputs("0x");
    char x[9];
    unsigned char i, c;

    x[8] = '\0';

    for (i = 0; i < 8; i++) {
        c = value & 0x0F;
        value >>= 4;
        c = (c >= 10) ? (c + 'A' - 10) : (c + '0');
        x[7 - i] = c;
    }

    UART_puts(x);
}

void UART_putdec32(unsigned long value) {
	
	char x[16];
	unsigned char i, c;

	x[sizeof(x) - 1] = 0;

	for(i = 0; i < sizeof(x) - 1; i++){
		
		c = (value % 10) + '0';
		value /= 10;
		x[sizeof(x) - 2 - i] = c;	
		if(!value) break;
	}

	UART_puts(x + sizeof(x) - 2 - i);
}

void UART_puthex64(unsigned long long value) {
    UART_pputs("0x");
    char x[17];
    unsigned char i, c;

    x[16] = '\0';

    for (i = 0; i < 16; i++) {
        c = value & 0x0F;
        value >>= 4;
        c = (c >= 10) ? (c + 'A' - 10) : (c + '0');
        x[15 - i] = c;
    }

    UART_puts(x);
}
