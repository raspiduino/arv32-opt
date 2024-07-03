#!/bin/bash

avr() {
    echo "Building for AVR"
    avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=main.lst  -std=gnu99 main.c -o main.o
    avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdcard.lst  -std=gnu99 sdcard.c -o sdcard.o
    avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdprint.lst  -std=gnu99 sdprint.c -o sdprint.o
    avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=spi.lst  -std=gnu99 spi.c -o spi.o
    avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=uart.lst  -std=gnu99 uart.c -o uart.o
    avr-gcc -mmcu=atmega328p -I. -DF_CPU=16000000UL -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99 main.o sdcard.o sdprint.o spi.o uart.o --output main.elf -Wl,-Map=main.map,--cref
    avr-objcopy -O ihex -R .eeprom main.elf main.hex
    avr-size -A -d main.elf
    echo "Done! Now flash the hex to your device (maybe using avrdude)"
}

sim() {
    echo "Building for simulation (debug)"
    gcc -I. -g3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99 main_sim.c if_sim.c -o sim
}

sim_opt() {
    echo "Building for simulation (release)"
    gcc -I. -march=native -Ofast -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99 main_sim.c if_sim.c -o sim
}

clean() {
    # sim
    rm sim dump.bin

    # avr
    rm *.o *.lst *.map *.elf
}

# Check for help flag
if [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
    echo "Usage: $0 (sim|sim_opt|avr)"
    echo "  sim: (Debug) Build computer simulation (exact same logic, only interfaces changed)"
    echo "  sim_opt: (Release) Build computer simulation (exact same logic, only interfaces changed)"
    echo "  avr: (Release) Build for AVR"
    exit 0
fi

# Check for arguments
if [[ "$1" == "sim" ]]; then
    sim
else
    if [[ "$1" == "sim_opt" ]]; then
        sim_opt
    else
        if [[ "$1" == "avr" ]]; then
            avr
        else
            if [[ "$1" == "clean" ]]; then
                clean
            else
                echo "Invalid argument: '$1'"
                exit 1
            fi
        fi
    fi
fi
