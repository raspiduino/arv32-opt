#ifndef __SD_PRINT_H__
#define __SD_PRINT_H__

#include <avr/io.h>

/* R1 MACROS */
#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

/* R2 MACROS */
#define OUT_OF_RANGE(X)     X & 0b10000000
#define ERASE_PARAM(X)      X & 0b01000000
#define WP_VIOLATION(X)     X & 0b00100000
#define CARD_ECC_FAILED(X)  X & 0b00010000
#define CC_ERROR(X)         X & 0b00001000
#define ERROR(X)            X & 0b00000100
#define WP_ERASE_SKIP(X)    X & 0b00000010
#define CARD_LOCKED(X)      X & 0b00000001

/* R3 MACROS */
#define POWER_UP_STATUS(X)  X & 0x40
#define CCS_VAL(X)          X & 0x40
#define VDD_2728(X)         X & 0b10000000
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

/* R7 MACROS */
#define CMD_VER(X)          ((X >> 4) & 0xF0)
#define VOL_ACC(X)          (X & 0x1F)
#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

/* DATA ERROR TOKEN */
#define SD_TOKEN_OOR(X)     X & 0b00001000
#define SD_TOKEN_CECC(X)    X & 0b00000100
#define SD_TOKEN_CC(X)      X & 0b00000010
#define SD_TOKEN_ERROR(X)   X & 0b00000001

void SD_printR1(uint8_t res);
void SD_printR2(uint8_t *res);
void SD_printR3(uint8_t *res);
void SD_printR7(uint8_t *res);
void SD_printBuf(uint8_t *buf);
void SD_printDataErrToken(uint8_t token);

#endif
