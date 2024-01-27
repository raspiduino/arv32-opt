#!/bin/bash

if [ "$1" == "" ] || [ "$2" == "" ]; then
  >&2 echo "Usage: $0 <microcontroller> <clock>"
  >&2 echo "If using an Uno, use \`$0 atmega328p 16000000\`"
  exit 1
fi

avr-gcc -c -mmcu="$1" -I. -DF_CPU="${2}UL"  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=main.lst  -std=gnu99 main.c -o main.o || exit 1
avr-gcc -c -mmcu="$1" -I. -DF_CPU="${2}UL"  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdcard.lst  -std=gnu99 sdcard.c -o sdcard.o || exit 1
avr-gcc -c -mmcu="$1" -I. -DF_CPU="${2}UL"  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdprint.lst  -std=gnu99 sdprint.c -o sdprint.o || exit 1
avr-gcc -c -mmcu="$1" -I. -DF_CPU="${2}UL"  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=spi.lst  -std=gnu99 spi.c -o spi.o || exit 1
avr-gcc -c -mmcu="$1" -I. -DF_CPU="${2}UL"  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=uart.lst  -std=gnu99 uart.c -o uart.o || exit 1
avr-gcc -mmcu="$1" -I. -DF_CPU="${2}UL" -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99 main.o sdcard.o sdprint.o spi.o uart.o --output main.elf -Wl,-Map=main.map,--cref || exit 1
avr-objcopy -O ihex -R .eeprom main.elf main.hex || exit 1
avr-size -A -d main.elf || exit 1
