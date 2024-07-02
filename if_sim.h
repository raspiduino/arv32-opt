#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "types.h"

// Config
#define RAM_SIZE 16777216UL         // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 1536               // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 1024        // Number of instructions executed before checking status. See loop()
#define TIME_DIVISOR 2
#define SD_BLOCK_LEN 512

void _delay_ms(uint32_t ms);
