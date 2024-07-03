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
#include <stdbool.h>
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

/*
 * Some notes about this stupid cache system
 * We are running on Arduino UNO, which uses atmega328p and only has 2KB of RAM
 * From what I measured, we could use up to 3x 512 bytes cache and there are
 * still enough RAN left for other stuff (local variables, etc)
 * So we will implement a system with 3 caches, each 512 bytes.
 *
 * Each cache has a 512 bytes buffer, a tag (the sector number of that 512-bytes
 * block in RAM), and a "dirty" flag (see below). In our implementation, we use
 * an uint16_t age score variables. For each data read, we will +1 to the age
 * score, and for each data write, we will also +1 to the age score.
 *
 * Cache will be invaild by LRU mechanism: when there is a read/write request to
 * an address that is not currently in the cache pool, a least recently used
 * cache will be invalid and a new 512-bytes block contains that address will be
 * fetch into the newly invaild cache. If that cache is marked as "dirty" (data
 * changed compared to RAM), it will be flushed to RAM first, then get invalid.
 *
 * When there is a write request to an address already in the cache pool, it
 * will follow lazy write mechanism. That means the change will be kept in the
 * cache, the cache will be marked as "dirty", and the changes won't be written
 * to main RAM until it gets invalid.
 *
 * When a cache is assigned as icache, it age score will be set to max (0xFFFF
 * for uint16_t). When the pc moves to an address that is not in the current
 * icache, it will get invalid, and its age score will be reset to 0. If pc
 * moves to an address that is currently in another dcache, that dcache will
 * be used for both icache and dcache, and its age score will be set to 0xFFFF.
 *
 * IMO, I don't think a cache will often be assigned as both icache and dcache
 * as the same time. It might just happend in self-modifying code.
 */

struct cache {
    uint8_t buf[SD_BLOCK_LEN];
    uint32_t tag;
    uint16_t age;
    bool flag; // False is dirty. Why not the opposite? AVR takes 2 instructions to store a 1 and just 1 to store a 0.
};

struct cache pool[3];

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
static Int32 HandleOtherCSRRead( UInt8 * image, UInt16 csrno );

// Load / store helper
static UInt32 store4(UInt32 ofs, UInt32 val);
static UInt16 store2(UInt32 ofs, UInt16 val);
static UInt8 store1(UInt32 ofs, UInt8 val);

static UInt32 load4(UInt32 ofs);
static UInt16 load2(UInt32 ofs);
static UInt8 load1(UInt32 ofs);

static UInt32 loadi(UInt32 ofs);

// Other
extern int __heap_start;
extern int *__brkval;
UInt32 last_cyclel = 0; // Last cyclel value
void dump_state(void);

// Config
const UInt32 RAM_SIZE = 16777216UL; // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 1536               // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024        // Number of instructions executed before checking status. See loop()
#define TIME_DIVISOR 2

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
#define MINIRV32_OTHERCSR_READ( csrno, value ) value = HandleOtherCSRRead( image, csrno );
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

// Init cache helper
void init_cache(UInt8 index, UInt32 tag, uint16_t age) {
    UInt8 token;
    UInt8 t = 0;

    // Read init sector
read_init_begin:
    SD_readSingleBlock(tag, pool[index].buf, &token); // buf = first sector
    if(!(token & 0xF0))
    {
        if (++t == 10) {
            UART_pputs("init_cache #");
            UART_puthex32(index);
            UART_pputs(" error, error token:\r\n");
            SD_printDataErrToken(token);
            UART_pputs("init_cache: failed 10 times in a row. Halting\r\n");
            while(1); // Halt
        }

        _delay_ms(100);
        goto read_init_begin;
    }

    // Set other info
    pool[index].tag = tag;
    pool[index].age = age;
    pool[index].flag = 0;
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

    // Setup cache
    // Init cache0 as icache
    init_cache(0, 0, 0xFFFF); // buf = 0x0 (code begin at 0x0)

    // Init cache1 as dcache
    init_cache(1, 6552, 0); // buf = second accessed address (got by dumping address)
    
    // Init cache2 as dcache
    init_cache(2, 3011, 0); // buf = third accessed address (got by dumping address)

    // Patch the ram to skip memory cleaning
    // We should patch it in the icache, not in the memory. Otherwise PC will jump to 0x0 for some reason
    // Maybe the code is actually copied to somewhere else
    // 0x00E6D863: Original opcode at 0xAC
    // 0x00E6C863: Opcode to skip the memory-cleaning loop. Got by disassembly
    // This works by changing BGE (branch greater or equal) to BLT (branch less than)
    pool[0].buf[0xAD] = 0xC8;

    // Set digital pin 9 to input pullup, see loop
    PORTB |= (1 << PINB1);

    // Init timer (from https://gist.github.com/adnbr/2439125)
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
    OCR1AH = 7; // Load the high byte of 2000
    OCR1AL = 208; // then the low byte into the output compare
    TIMSK1 |= (1 << OCIE1A); // Enable the compare match interrupt
    sei(); // Now enable global interrupts

    // Print current free memory
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
static uint32_t HandleException( uint32_t ir, uint32_t code )
{
	// Weird opcode emitted by duktape on exit.
	if( code == 3 )
	{
		// Could handle other opcodes here.
	}
	return code;
}

static uint32_t HandleControlStore( uint32_t addy, uint32_t val )
{
	if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
	{
		UART_putc(val);
	}
	return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
	// Emulating a 8250 / 16550 UART
	if( addy == 0x10000005 )
		return 0x60 | UART_available();
	else if( addy == 0x10000000 && UART_available() )
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
	else if( csrno == 0x138 )
	{
		//Print "string"
		uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
		uint32_t ptrend = ptrstart;
		if( ptrstart >= RAM_SIZE ) {
            UART_pputs( "DEBUG PASSED INVALID PTR (");
            UART_puthex32(value);
            UART_pputs(")\r\n");
        }
		while( ptrend < RAM_SIZE )
		{
			if( load1(ptrend) == 0 ) break;
			ptrend++;
		}
		if( ptrend != ptrstart ) {
            for (; ptrstart <= ptrend; ptrstart++) {
                UART_putc(load1(ptrstart));
            }
        }
	}
	else if( csrno == 0x139 )
	{
		UART_putc((UInt8)value);
	}
}

static Int32 HandleOtherCSRRead( UInt8 * image, UInt16 csrno )
{
	if( csrno == 0x140 )
	{
		if( !UART_available() ) return -1;
		return UART_getc();
	}
	return 0;
}

// Cache helper functions

/*
 * flag:
 * false -> data fetch, calculate sector
 * true -> instruction fetch, calculate sector
 *
 * return: cache buffer's address
 *
 * quick note: storing a 0 takes just a single instruction, but storing a 1 takes 2 on AVR
 */

uint8_t *read_buf(uint32_t ofs, bool flag, bool write) {
    // Calculate sector num
    // Dividing on AVR is a pain, so we should avoid that if we can
    uint32_t s = ofs / SD_BLOCK_LEN;

    // Check if the requested sector exists in the pool
    // We have 3 caches, so we can use if. If you implement more than 3 caches, you should
    // use for loop
    uint8_t ret = 0;
    if (s == pool[0].tag) {
        ret = 0;
        
        // Add age
        if (pool[0].age <= 0xFFFE) {
            pool[0].age += 1;
        }

        // Set dirty flag if needed
        if (write) {
            pool[0].flag = false; // false = dirty
        }
    } else if (s == pool[1].tag) {
        ret = 1;
        
        // Add age
        if (pool[1].age <= 0xFFFE) {
            pool[1].age += 1;
        }

        // Set dirty flag if needed
        if (write) {
            pool[1].flag = false;
        }
    } else if (s == pool[2].tag) {
        ret = 2;

        // Add age
        if (pool[2].age <= 0xFFFE) {
            pool[2].age += 1;
        }

        // Set dirty flag if needed
        // If you think this can be optimized to pool[2].flag = write, think again!
        if (write) {
            pool[2].flag = false;
        }
    } else {
        uint8_t lru = 2;

        if (flag) {
            // If icache miss -> invaild old icache
            if (pool[0].age == 0xFFFF) {
                lru = 0;
                pool[0].age = 0;
                goto continue_without_finding_lru;
            } else if (pool[1].age == 0xFFFF) {
                lru = 1;
                pool[1].age = 0;
                goto continue_without_finding_lru;
            } else {
                lru = 2;
                pool[2].age = 0;
                goto continue_without_finding_lru;
            }
        }

        // Cache miss -> invalid LRU and fetch new
        // Find LRU cache
        if (pool[lru].age > pool[1].age) {
            lru = 1;
        }

        if (pool[lru].age > pool[0].age) {
            lru = 0;
        }

continue_without_finding_lru:
        // Check if LRU cache if dirty
        uint8_t token;
        if (!pool[lru].flag) { // false = dirty
            // Dirty -> flush to SD
            uint8_t t = 0;
cache_write:
            SD_writeSingleBlock(pool[lru].tag, pool[lru].buf, &token);
            if (!(token == SD_DATA_ACCEPTED)) {
                if(!(token & 0xF0))
                {
                    if (++t == 10) {
                        UART_pputs("cache_write @ ");
                        UART_puthex32(pool[lru].tag);
                        UART_pputs(" error, error token:\r\n");
                        SD_printDataErrToken(token);
                        dump_state();
                        UART_pputs("cache_write: failed 10 times in a row. Halting\r\n");
                        while(1); // Halt
                    }

                    _delay_ms(100);
                    goto cache_write;
                }
            }

            // Clear dirty flag
            pool[lru].flag = true; // true = not dirty
        }

        // Set new properties
        pool[lru].tag = s;
        if (flag) {
            // icache
            pool[lru].age = 0xFFFF;
#ifdef ENABLE_CACHE_STAT
            icache_miss++;
#endif
        } else {
            // dcache
            pool[lru].age = 1; // Already +1 for the access
#ifdef ENABLE_CACHE_STAT
            dcache_miss++;
#endif
        }

        // Set dirty flag if needed
        if (write) {
            pool[lru].flag = false;
        }
        
        // Fetch new sector into cache
        uint8_t t = 0;
cache_read:
        SD_readSingleBlock(s, pool[lru].buf, &token);
        if (!(token == SD_START_TOKEN)) {
            if(!(token & 0xF0)) {
                if (++t == 10) {
                    UART_pputs("cache_read @ ");
                    UART_puthex32(s);
                    UART_pputs(" error, error token:\r\n");
                    SD_printDataErrToken(token);
                    dump_state();
                    UART_pputs("cache_read: failed 10 times in a row. Halting\r\n");
                    while(1); // Halt
                }

                _delay_ms(100);
                goto cache_read;
            }
        }

        // Return buffer address
        return &pool[lru].buf;
    }

    // Cache hit
#ifdef ENABLE_CACHE_STAT
    if (flag) {
        icache_hit++;
    } else {
        dcache_hit++;
    }
#endif

    // Return buffer address
    return &pool[ret].buf;
}

// Memory access functions
static uint32_t loadi(uint32_t ofs) {
    // Load instruction from icache
    uint16_t r = ofs % 512; // Don't inline this. Clearly specify type for compiler to optimize. Else it will be seen as uint32_t
    //return *(uint32_t*)&pool[id].buf[(uint16_t)(ofs % 512)];
    return *(uint32_t*)(read_buf(ofs, true, false) + r);
}

static uint32_t load4(uint32_t ofs) {
    uint16_t r = ofs % 512; // Don't inline this
    //return *(uint32_t*)&pool[id].buf[r];
    return *(uint32_t*)(read_buf(ofs, false, false) + r);
}
static uint16_t load2(uint32_t ofs) {
    uint16_t r = ofs % 512;
    //return *(uint16_t *)&pool[id].buf[r];
    return *(uint16_t*)(read_buf(ofs, false, false) + r);
}
static uint8_t load1(uint32_t ofs) {
    uint16_t r = ofs % 512;
    //return pool[id].buf[(uint16_t)(ofs % 512)];
    return *(uint8_t*)(read_buf(ofs, false, false) + r);
}

static uint32_t store4(uint32_t ofs, uint32_t val) {
    uint16_t r = ofs % 512;

    // Store
    //*(uint32_t *)&pool[id].buf[r] = val;
    *(uint32_t *)(read_buf(ofs, false, true) + r) = val;

    // Return result
    return val;
}

static uint16_t store2(uint32_t ofs, uint16_t val) {
    uint16_t r = ofs % 512;

    // Store
    //*(uint16_t *)&pool[id].buf[r] = val;
    *(uint16_t *)(read_buf(ofs, false, true) + r) = val;

    // Return result
    return val;
}

static uint8_t store1(uint32_t ofs, uint8_t val) {
    uint16_t r = ofs % 512;

    // Store
    //*(uint8_t *)&pool[id].buf[r] = val;
    *(uint8_t *)(read_buf(ofs, false, true) + r) = val;

    // Return results
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
