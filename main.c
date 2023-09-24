/*
 * arv32-opt: mini-rv32ima on Arduino UNO, but the code is written in pure C
 * Created by gvl610
 * mini-rv32ima is created by cnlohr. See https://github.com/cnlohr/mini-rv32ima
 * UART, SPI, and SD code is created by ryanj1234. See https://github.com/ryanj1234/SD_TUTORIAL_PART4
 */

/*
 * Pinout on Arduino UNO:
 *  SD       Arduino UNO
 *  CS           10
 * MOSI          11
 * MISO          12
 * SCLK          13
 *
 * You can change the CS pin by modifying spi.h
 */

// Frequency that atmega328p is running on. Default is 16MHz on Arduino UNO board
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// UART baudrate. Default is 9600
#define BAUD_RATE 9600

// Enable/disable cache hit/miss information
#define ENABLE_CACHE_STAT

// Headers
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"
#include "spi.h"
#include "sdcard.h"
#include "sdprint.h"
#include "types.h"

// mini-rv32ima variables
int fail_on_all_faults = 0;
uint64_t lastTime = 0;
struct MiniRV32IMAState *core;

// SD access variables
UInt8 icache[SD_BLOCK_LEN];
UInt8 dcache[SD_BLOCK_LEN];
UInt32 last_sector = 0;
UInt32 last_sectori = 0;

// Cache hit / miss stat
#ifdef ENABLE_CACHE_STAT
UInt32 icache_hit = 0;
UInt32 icache_miss = 0;
UInt32 dcache_hit = 0;
UInt32 dcache_miss = 0;
#endif

// Functions prototype
static UInt32 HandleException( UInt32 ir, UInt32 retval );
static UInt32 HandleControlStore( UInt32 addy, UInt32 val );
static UInt32 HandleControlLoad( UInt32 addy );
static void HandleOtherCSRWrite( UInt8 * image, UInt16 csrno, UInt32 value );

// Load / store helper
static UInt32 store4(UInt32 ofs, UInt32 val);
static UInt16 store2(UInt32 ofs, UInt16 val);
static UInt8 store1(UInt32 ofs, UInt8 val);

static UInt32 load4(UInt32 ofs);
static UInt16 load2(UInt32 ofs);
static UInt8 load1(UInt32 ofs);

static UInt32 loadi(UInt32 ofs);

// Other
UInt32 last_cyclel = 0; // Last cyclel value
void dump_state(void);

// Config
const UInt32 RAM_SIZE = 12582912UL; // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 1536               // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024        // Number of instructions executed before checking status. See loop()
#define TIME_DIVISOR 1

// Setup mini-rv32ima
// This is the functionality we want to override in the emulator.
// think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN( x... ) UART_pputs( x );
#define MINIRV32_DECORATE  static
#define MINI_RV32_RAM_SIZE RAM_SIZE
#define MINIRV32_IMPLEMENTATION // Minimum rv32 emulator
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { UART_pputs("FAULT\r\n"); return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );
#define MINIRV32_CUSTOM_MEMORY_BUS // Custom RAM handler for swapping to SD card

// Macro for accessing RAM
#define MINIRV32_STORE4( ofs, val ) store4(ofs, val)
#define MINIRV32_STORE2( ofs, val ) store2(ofs, val)
#define MINIRV32_STORE1( ofs, val ) store1(ofs, val)
#define MINIRV32_LOAD4( ofs ) load4(ofs)
#define MINIRV32_LOAD2_SIGNED( ofs ) (Int8)load2(ofs)
#define MINIRV32_LOAD2( ofs ) load2(ofs)
#define MINIRV32_LOAD1_SIGNED( ofs ) (Int8)load1(ofs)
#define MINIRV32_LOAD1( ofs ) load1(ofs)
#define MINIRV32_LOADI( ofs ) loadi(ofs)

#include "mini-rv32ima.h"

// millis implementation from https://gist.github.com/adnbr/2439125
volatile unsigned long timer1_millis;
unsigned long last_ms = 0;
 
ISR (TIMER1_COMPA_vect) {
    timer1_millis++;
}

unsigned long millis(void) {
    unsigned long millis_return;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        millis_return = timer1_millis;
    }

    return millis_return;
}

// Entry point
int main(void) {
    // Initialize UART
    UART_init();

    // Say something, so people know that UART works
    UART_pputs("arv32-opt: mini-rv32ima on Arduino UNO\r\n");

    // Initialize SPI
    SPI_init(SPI_MASTER | SPI_FOSC_16 | SPI_MODE_0);

    // Initialize SD card
    if(SD_init() != SD_SUCCESS)
    {
        UART_pputs("Error initializaing SD card\r\n");
        while(1); // Deadloop
    }

    UART_pputs("SD card initialized successfully!\r\n");

    // Initialize emulator struct
    core = (struct MiniRV32IMAState *)malloc(sizeof(struct MiniRV32IMAState));
    memset(core, 0, sizeof(struct MiniRV32IMAState));

    // Setup core
    core->pc = MINIRV32_RAM_IMAGE_OFFSET;
    core->regs[10] = 0x00; //hart ID
    core->regs[11] = RAM_SIZE - sizeof(struct MiniRV32IMAState) - DTB_SIZE + MINIRV32_RAM_IMAGE_OFFSET; // dtb_pa (Must be valid pointer) (Should be pointer to dtb)
    core->extraflags |= 3; // Machine-mode.

    // Prefetch first sector to dcache and icache
    UInt8 token;
    SD_readSingleBlock(0, dcache, &token);
    memcpy(icache, dcache, SD_BLOCK_LEN);

    // Patch the ram to skip memory cleaning
    // We should patch it in the icache, not in the memory. Otherwise PC will jump to 0x0 for some reason
    // Maybe the code is actually copied to somewhere else
    // 0xFEE6CCE3: Original opcode at 0xB9
    // 0xFEE6DCE3: Opcode to skip the memory-cleaning loop. Got by disassembling the RAM dump)
    // This works by changing BLT (branch less than) to BGE (branch greater or equal)
    icache[0xB9] = 0xDC;

    // Set digital pin 9 to input pullup, see loop
    PORTB |= (1 << PINB1);

    // Init timer (from https://gist.github.com/adnbr/2439125)
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
    OCR1AH = 7; // Load the high byte of 2000
    OCR1AL = 208; // then the low byte into the output compare
    TIMSK1 |= (1 << OCIE1A); // Enable the compare match interrupt
    sei(); // Now enable global interrupts

    // Print current free memory
    extern int __heap_start;
    extern int *__brkval;
    UART_pputs("Current AVR free memory: ");
    UART_putdec32((int) SP - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
    UART_pputs(" bytes\r\n");

    // Emulator loop
    // It's stated in the AVR documentation that writing do ... while() is faster than while()
    do {
        // Print processor state if requested by user
        if (!(PINB & (1 << PINB1))) {
            // Calculate effective emulated speed
            unsigned long current_ms = millis();
            UART_pputs("Effective emulated speed: ");
            UART_putdec32(((core->cyclel - last_cyclel) * 1000) / (current_ms - last_ms));
            UART_pputs(" Hz, dtime=");
            UART_putdec32(current_ms - last_ms);
            UART_pputs("ms, dcycle=");
            UART_putdec32(core->cyclel - last_cyclel);
            UART_pputs("\r\n");

            // Print current free memory
            UART_pputs("Current AVR free memory: ");
            UART_putdec32((int) SP - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
            UART_pputs(" bytes\r\n");

            // Print icache/dcache hit/miss
#ifdef ENABLE_CACHE_STAT
            UART_pputs("icache hit/miss: ");
            UART_putdec32(icache_hit);
            UART_pputs("/");
            UART_putdec32(icache_miss);
            UART_pputs("; dcache hit/miss: ");
            UART_putdec32(dcache_hit);
            UART_pputs("/");
            UART_putdec32(dcache_miss);
            UART_pputs("\r\n");
#endif
            // Dump state
            dump_state();
            UART_pputs("Dump completed. Emulator will continue when B1 is set back to HIGH\r\n");

            // Wait until B1 is set back to HIGH
            while (!(PINB & (1 << PINB1)));
            UART_pputs("B1 is set to HIGH, emulator resume\r\n");
            
            // Reset counters
            last_cyclel = core->cyclel;
            last_ms = millis();
        }

        // Calculate pseudo time
        uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
        UInt32 elapsedUs = 0;
        elapsedUs = *this_ccount / TIME_DIVISOR - lastTime;
        lastTime += elapsedUs;

        int ret = MiniRV32IMAStep( core, NULL, 0, elapsedUs, INSTRS_PER_FLIP ); // Execute upto INSTRS_PER_FLIP cycles before breaking out.
        switch( ret )
        {
            case 0: break;
            case 1: _delay_ms(1); *this_ccount += INSTRS_PER_FLIP; break;
            //case 3: instct = 0; break;
            //case 0x7777: goto restart;  //syscon code for restart
            case 0x5555: UART_pputs("POWEROFF\r\n"); while(1); //syscon code for power-off . halt
            default: UART_pputs("Unknown failure\r\n"); break;
        }
    } while (1);

    // Should not get here
    while(1);
}

// Exception handlers
static UInt32 HandleException( UInt32 ir, UInt32 code )
{
    // Weird opcode emitted by duktape on exit.
    if( code == 3 )
    {
        // Could handle other opcodes here.
    }
    return code;
}

static UInt32 HandleControlStore( UInt32 addy, UInt32 val )
{
    if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
    {
        UART_putc((char)val);
    }
    
    return 0;
}

static UInt32 HandleControlLoad( UInt32 addy )
{
    // Emulating a 8250 / 16550 UART
    if ( addy == 0x10000005 )
        return 0x60 | UART_available();
    else if ( addy == 0x10000000 && (UART_available()) )
        return UART_getc();
    return 0;
}

static void HandleOtherCSRWrite( UInt8 * image, UInt16 csrno, UInt32 value )
{
    if( csrno == 0x136 )
    {
        UART_putdec32(value);
    }

    if( csrno == 0x137 )
    {
        UART_puthex32(value);
    }

    /*else if( csrno == 0x138 )
    {
        // Print "string"
        UInt32 ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
        UInt32 ptrend = ptrstart;
        if( ptrstart >= ram_amt )
        printf( "DEBUG PASSED INVALID PTR (%08x)\n", value );
        while( ptrend < ram_amt )
        {
        if( image[ptrend] == 0 ) break;
        ptrend++;
        }
        if( ptrend != ptrstart )
        fwrite( image + ptrstart, ptrend - ptrstart, 1, stdout );
    }*/
}

void read_buf(UInt32 ofs, UInt8 flag) {
    uint32_t s;

    /*
     * Some operations might involve read/write bytes that are located between 2 sectors on
     * the SD card. In that case, we have to fetch 2 continuous sectors at a time. Since
     * we already know the last sector number, we can just read the n + 1 sector and skip
     * the division (which is a pain on AVR). The flag parameter is used for this. When
     * flag is 0, we will calculate the sector number. Else, we will just fetch the n + 1
     * sector.
     */
    if (flag == 0) {
        // Calculate sector num
        // Dividing on AVR is a pain, so we should avoid that if we can
        s = ofs / SD_BLOCK_LEN;

        // If sector num the same as last one, then return because we already read that sector
        if (s == last_sector) {
#ifdef ENABLE_CACHE_STAT
            dcache_hit++;
#endif
            return;
        } else {
            last_sector = s;
        }
    } else {
        // Fetch n + 1 sector
        s = ++last_sector;
    }

#ifdef ENABLE_CACHE_STAT
    dcache_miss++;
#endif

    UInt8 res, token;
    UInt8 t = 0;
read_begin:
    res = SD_readSingleBlock(s, dcache, &token);

    // If no error -> return
    if((res == 0x00) && (token == SD_START_TOKEN))
        return;
    // Else if error token received, print
    else if(!(token & 0xF0))
    {
        if (++t == 10) {
            UART_pputs("read_buf @ ");
            UART_puthex32(ofs);
            UART_pputs(" error, error token:\r\n");
            SD_printDataErrToken(token);
            dump_state();
            UART_pputs("read_buf: failed 10 times in a row. Halting\r\n");
            while(1); // Halt
        }

        _delay_ms(100);
        goto read_begin;
    }
}

void read_bufi(UInt32 ofs, UInt8 flag) {
    uint32_t s;

    /*
     * Some operations might involve read/write bytes that are located between 2 sectors on
     * the SD card. In that case, we have to fetch 2 continuous sectors at a time. Since
     * we already know the last sector number, we can just read the n + 1 sector and skip
     * the division (which is a pain on AVR). The flag parameter is used for this. When
     * flag is 0, we will calculate the sector number. Else, we will just fetch the n + 1
     * sector.
     */
    if (flag == 0) {
        // Calculate sector num
        // Dividing on AVR is a pain, so we should avoid that if we can
        s = ofs / SD_BLOCK_LEN;

        // If sector num the same as last one, then return because we already read that sector
        // Flush cache if required
        if (s == last_sectori) {
#ifdef ENABLE_CACHE_STAT
            icache_hit++;
#endif
            return;
        } else {
            last_sectori = s;
        }
    } else {
        // Fetch n + 1 sector
        s = ++last_sectori;
    }

#ifdef ENABLE_CACHE_STAT
    icache_miss++;
#endif

    UInt8 res, token;
    UInt8 t = 0;
readi_begin:
    res = SD_readSingleBlock(s, icache, &token);

    // If no error -> return
    if((res == 0x00) && (token == SD_START_TOKEN))
        return;
    // Else if error token received, print
    else if(!(token & 0xF0))
    {
        if (++t == 10) {
            UART_pputs("read_buf @ ");
            UART_puthex32(ofs);
            UART_pputs(" error, error token:\r\n");
            SD_printDataErrToken(token);
            dump_state();
            UART_pputs("read_bufi: failed 10 times in a row. Halting\r\n");
            while(1); // Halt
        }

        _delay_ms(100);
        goto readi_begin;
    }
}

void write_buf(void) {
    UInt8 res, token;
    UInt8 t = 0;

    // Set flush icache if needed
    if (last_sector == last_sectori) {
        // If we write in the same sector as icache, then we need to flush icache to support self-modifying code
        memcpy(icache, dcache, SD_BLOCK_LEN);
    }

write_begin:
    // Write to last sector (since you already read last sector, then modify the content before you can write)
    res = SD_writeSingleBlock(last_sector, dcache, &token);

    // If no error -> return
    if((res == 0x00) && (token == SD_DATA_ACCEPTED))
        return;
    // Else if error token received, print
    else if(!(token & 0xF0))
    {
        if (++t == 10) {
            UART_pputs("write_buf @ sector ");
            UART_puthex32(last_sector);
            UART_pputs(" error, error token:\r\n");
            SD_printDataErrToken(token);
            dump_state();
            UART_pputs("write_buf: failed 10 times in a row. Halting\r\n");
            while(1); // Halt
        }

        _delay_ms(100);
        goto write_begin;
    }
}

// Memory access functions

static UInt32 loadi(UInt32 ofs) {
    // Load instruction from icache

    // No longer needed since it's now done in ram file
    /*if (ofs == 0xB8) {
        // Skip memory clean
        UART_pputs("Currently at 0xB8\r\n");
        if (dh == 1) {while(1);}
        dh = 1;
        return 0xFEE6DCE3; // RISC-V opcode to skip the memory-cleaning loop. Got by disassembling the RAM dump
    }*/

    UInt32 result;
    UInt16 r = ofs % 512;
    read_bufi(ofs, 0);
    if (r >= 509) {
        // 1 - 3 bytes are in nth sector, and the others in n + 1 sector
        // Read the nth sector and get the bytes in that sector
        UInt8 i = 0;
        for (; i < SD_BLOCK_LEN - r; i++) {
            ((UInt8 *)&result)[i] = icache[r + i];
        }

        // Read the next sector and get the remaining bytes
        read_buf(ofs, 1);
        for (UInt8 j = 0; j < r - 508; j++) {
            ((UInt8 *)&result)[i + j] = icache[j];
        }

        return result;
    }

    ((UInt8 *)&result)[0] = icache[r];     // LSB
    ((UInt8 *)&result)[1] = icache[r + 1];
    ((UInt8 *)&result)[2] = icache[r + 2];
    ((UInt8 *)&result)[3] = icache[r + 3]; // MSB
    return result;
}

static UInt32 load4(UInt32 ofs) {
    UInt32 result;

    // No longer needed since it's now done in ram file
    /*if (ofs == 0xB8) {
        // Skip memory clean
        UART_pputs("Currently at 0xB8\r\n");
        if (dh == 1) {while(1);}
        dh = 1;
        return 0xFEE6DCE3; // RISC-V opcode to skip the memory-cleaning loop. Got by disassembling the RAM dump
    }*/

    UInt16 r = ofs % 512;
    read_buf(ofs, 0);
    if (r >= 509) {
        // 1 - 3 bytes are in nth sector, and the others in n + 1 sector
        // Read the nth sector and get the bytes in that sector
        UInt8 i = 0;
        for (; i < SD_BLOCK_LEN - r; i++) {
            ((UInt8 *)&result)[i] = dcache[r + i];
        }

        // Read the next sector and get the remaining bytes
        read_buf(ofs, 1);
        for (UInt8 j = 0; j < r - 508; j++) {
            ((UInt8 *)&result)[i + j] = dcache[j];
        }

        return result;
    }

    ((UInt8 *)&result)[0] = dcache[r];     // LSB
    ((UInt8 *)&result)[1] = dcache[r + 1];
    ((UInt8 *)&result)[2] = dcache[r + 2];
    ((UInt8 *)&result)[3] = dcache[r + 3]; // MSB
    return result;
}
static UInt16 load2(UInt32 ofs) {
    UInt16 result;
    UInt16 r = ofs % 512;
    read_buf(ofs, 0);
    if (r == 511) {
        // LSB located in nth sector
        ((UInt8 *)&result)[0] = dcache[511];

        // MSB located in n + 1 sector
        read_buf(ofs, 1);
        ((UInt8 *)&result)[1] = dcache[0];

        return result;
    }

    ((UInt8 *)&result)[0] = dcache[r];     // LSB
    ((UInt8 *)&result)[1] = dcache[r + 1]; // MSB
    return result;
}
static UInt8 load1(UInt32 ofs) {
  read_buf(ofs, 0);
  return dcache[ofs % 512];
}

static UInt32 store4(UInt32 ofs, UInt32 val) {
    UInt16 r = ofs % 512;
    read_buf(ofs, 0);
    if (r >= 509) {
        // 1 - 3 bytes are in nth sector, and the others in n + 1 sector
        // Read the nth sector and change the bytes in that sector
        UInt8 i = 0;
        for (; i < SD_BLOCK_LEN - r; i++) {
            dcache[r + i] = ((UInt8 *)&val)[i];
        }

        // Write back to that sector
        write_buf();

        // Read the next sector and get the remaining bytes
        read_buf(ofs, 1);
        for (UInt8 j = 0; j < r - 508; j++) {
            dcache[j] = ((UInt8 *)&val)[i + j];
        }

        // Write back to that sector
        write_buf();
        return val;
    }

    dcache[r]     = ((UInt8 *)&val)[0]; // LSB
    dcache[r + 1] = ((UInt8 *)&val)[1];
    dcache[r + 2] = ((UInt8 *)&val)[2];
    dcache[r + 3] = ((UInt8 *)&val)[3]; // MSB
    write_buf();
    return val;
}

static UInt16 store2(UInt32 ofs, UInt16 val) {
    UInt16 r = ofs % 512;
    read_buf(ofs, 0);

    if (r == 511) {
        // LSB located in the nth sector
        dcache[511] = ((UInt8 *)&val)[0];
        write_buf();

        // MSB located in the n + 1 sector
        read_buf(ofs, 1);
        dcache[0] = ((UInt8 *)&val)[1];
        write_buf();
        return val;
    }

    dcache[r]     = ((UInt8 *)&r)[0]; // LSB
    dcache[r + 1] = ((UInt8 *)&r)[1]; // MSB
    write_buf();
    return val;
}

static UInt8 store1(UInt32 ofs, UInt8 val) {
    read_buf(ofs, 0);
    dcache[ofs % 512] = val;
    write_buf();
    return val;
}

void dump_state(void) {
    UART_pputs("==============================================================================\r\n");
    UART_pputs("Dumping emulator state:\r\nRegisters x0 - x31:\r\n");

    // Print registers
    for (uint8_t i = 0; i < 8; i++) {
        // Print 4 registers at once
        UART_puthex32(core->regs[i*4]);     UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 1]); UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 2]); UART_pputs(" ");
        UART_puthex32(core->regs[i*4 + 3]); UART_pputs("\r\n");
    }

    UART_pputs("pc: ");
    UART_puthex32(core->pc);
    UART_pputs("\r\nmstatus: ");
    UART_puthex32(core->mstatus);
    UART_pputs("\r\ncyclel: ");
    UART_puthex32(core->cyclel);
    UART_pputs("\r\ncycleh: ");
    UART_puthex32(core->cycleh);
    UART_pputs("\r\ntimerl: ");
    UART_puthex32(core->timerl);
    UART_pputs("\r\ntimerh: ");
    UART_puthex32(core->timerh);
    UART_pputs("\r\ntimermatchl: ");
    UART_puthex32(core->timermatchl);
    UART_pputs("\r\ntimermatchh: ");
    UART_puthex32(core->timermatchh);
    UART_pputs("\r\nmscratch: ");
    UART_puthex32(core->mscratch);
    UART_pputs("\r\nmtvec: ");
    UART_puthex32(core->mtvec);
    UART_pputs("\r\nmie: ");
    UART_puthex32(core->mie);
    UART_pputs("\r\nmip: ");
    UART_puthex32(core->mip);
    UART_pputs("\r\nmepc: ");
    UART_puthex32(core->mepc);
    UART_pputs("\r\nmtval: ");
    UART_puthex32(core->mtval);
    UART_pputs("\r\nmcause: ");
    UART_puthex32(core->mcause);
    UART_pputs("\r\nextraflags: ");
    UART_puthex32(core->extraflags);
    UART_pputs("\r\n==============================================================================\r\n");
}
