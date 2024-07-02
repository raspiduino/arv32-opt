#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/time.h>

#include "if_sim.h"

uint8_t mem[RAM_SIZE];

// UART
const char hex_digits[] = "0123456789ABCDEF";
static int is_eofd;

void UART_init()
{
    return; // Do literally nothing
}

void UART_putc(const unsigned char data)
{
	putchar(data);
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
	if( is_eofd ) return 0xffffffff;
	char rxchar = 0;
	int rread = read(fileno(stdin), (char*)&rxchar, 1);

	if( rread > 0 ) // Tricky: getchar can't be used with arrow keys.
		return rxchar;
	else
		return -1;
}

unsigned char UART_available(void)
{
	if( is_eofd ) return -1;
	int byteswaiting;
	ioctl(0, FIONREAD, &byteswaiting);
	if( !byteswaiting && write( fileno(stdin), 0, 0 ) != 0 ) { is_eofd = 1; return -1; } // Is end-of-file for 
	return !!byteswaiting;
}

void UART_puthex32(unsigned long value) {
    UART_puts("0x");
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

// SPI
void SPI_init(uint16_t initParams) {
    // Do literally nothing
}

// SD
void SD_printDataErrToken(uint8_t token) {
    // Literally impossible in a simulation, but just in case
    printf("SD error: %x", token);
}

uint8_t SD_init() {
    // Open and read RAM file to memory array
    FILE *fp = fopen("rv32.bin", "rb");
    if (fp == NULL) {
        perror("fopen");
        return 1;
    }

    // Check if file size matches expected size
    fseek(fp, 0, SEEK_END); // Seek to the end of the file
    long actual_size = ftell(fp); // Get the file size
    rewind(fp); // Rewind to the beginning for reading

    if (actual_size != RAM_SIZE) {
        fprintf(stderr, "Warning: File size (%ld bytes) does not match expected size (%d bytes)\n", actual_size, RAM_SIZE);
    }

    // Read the entire file into the buffer
    size_t bytes_read = fread(mem, 1, RAM_SIZE, fp);
    if (bytes_read != RAM_SIZE) {
        fprintf(stderr, "fread error: %zu bytes read (expected %d)\n", bytes_read, RAM_SIZE);
        fclose(fp);
        return 1;
    }

    return 0; // Success
}

uint8_t SD_readSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *token) {
    // Read
    memcpy(buf, mem + (addr * SD_BLOCK_LEN), 512);

    // Read success
    *token = 0xFF;
}

uint8_t SD_writeSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *res) {
    // Write
    memcpy(mem + (addr * SD_BLOCK_LEN), buf, 512);

    // Write success
    *res = 0xFF;
}

// Misc
void _delay_ms(unsigned int ms) {
    usleep(ms * 1000);
}
