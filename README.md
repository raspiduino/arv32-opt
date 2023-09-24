# arv32-opt
[Still testing!] Atmega328p port of mini-rv32ima. Let's run Linux on the world's worst Linux PC (and beat Dmitry Grinberg)

## What is this?
This is a port of [mini-rv32ima](https://github.com/cnlohr/mini-rv32ima) (a minimum RISC-V emulator, capable of booting Linux) on atmega328p (the core of Arduino UNO, a 8-bit AVR microcontroller). So basically, this code is for **booting Linux on Arduino UNO**.

Yes you are reading it correctly, Arduino UNO can (theorically, but not practically) boot Linux. And it definitely beats [Dmitry Grinberg's (once) world's worst Linux PC](https://dmitry.gr/?r=05.Projects&proj=07.%20Linux%20on%208bit).

## How does it work?
The idea is really simple: you have an Arduino UNO (or atmega328p) to run the emulator's logic, and emulator's RAM is accessed via swapping with an SD card (which is communicated through SPI interface, see more below).

The code is written in pure C (and not Arduino) to reduce Arduino overhead (if any). It initializes UART, SPI, SD card, and a digital input-pullup pin for triggering emulator state dump. Finally, it initialize mini-rv32ima and let the emulator does its works.

## How fast is it?
About ~~175Hz - 205Hz~~ 535 - 600 Hz with `-O3` code on an Arduino UNO based on atmega328p, clocked at 16MHz, with a class 4 SDHC card connected via 1-bit SPI interface. The time for fully booting Linux is estimated to be ~~`119.4` hours, or `4.96` days~~ `39.1` hours, or `1.63` day, if calculated correctly.

Update 24/9/2023: The speed is tripled by implementing icache

<br> Why it's *that* slow? Read `Current issues and drawbacks` section below.

## Pinout
The pinout is really simple. On Arduino UNO, it should be:

|Arduino UNO pin|Connect to|Description|
|---------------|----------|-----------|
|9|ground-connected button|When connected to GND, emulator state and effective emulated speed will be dump via UART|
|10|`CS` pin on SD card|SD's chip select/`CS`/`EN`/`SS` pin|
|11|`MOSI` pin on SD card|SD's `DI`/`CMD` pin|
|12|`MISO` pin on SD card|SD's `DO` pin|
|13|`SCLK` pin on SD card|SD's clock/`SCK`/`CLK` pin|

> [!WARNING]
> Since Arduino UNO (atmega328p) uses **5V** logic level, while SD card use **3.3V (or lower)** logic level, you will need a **level shifter**. You can build your own using MOSFET, but the simplest way is to buy an SD card adapter.

> [!NOTE]  
> You can change the `CS` and the pin used to dump state to any pins you want. Just modify the code (see below).

## Usage
### Preparing the SD card
- First, you will need an SD card. Any type larger than 12MB.
> [!WARNING]
> You should backup all your files, since doing this SD card preperation will destroy your SD card's filesystem.
- Then you need some tool to directly write a file to SD card. On Windows, you can use [HDD Raw Copy Tool](https://hddguru.com/software/HDD-Raw-Copy-Tool/), on *nix you can use `dd if=file.bin of=/dev/sdX conv=notrunc`.
- Download [this](https://github.com/raspiduino/mini-rv32ima-swap/raw/main/dump.org.bin) file.
- Write it directly to the SD card using the tools in step 2.
- Now you are done preparing the card.

> [!IMPORTANT]  
> The `Preparing the SD card` section must be repeated every time you boot your emulator. Otherwise it might not boot (if Linux has initialized enough and start cleaning the memory). This might be fixed in the future.

### Building the code
You can just download the latest HEX file in [Release](https://github.com/raspiduino/arv32-opt/releases/). But if you want to build on your own, here are the steps:
- First, clone the repo: `git clone https://github.com/raspiduino/arv32-opt`. Then `cd arv32-opt`
- Then use build the code using the following commands:
```cmd
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=main.lst  -std=gnu99 main.c -o main.o
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdcard.lst  -std=gnu99 sdcard.c -o sdcard.o
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=sdprint.lst  -std=gnu99 sdprint.c -o sdprint.o
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=spi.lst  -std=gnu99 spi.c -o spi.o
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=uart.lst  -std=gnu99 uart.c -o uart.o
avr-gcc -mmcu=atmega328p -I. -DF_CPU=16000000UL -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -std=gnu99 main.o sdcard.o sdprint.o spi.o uart.o --output main.elf -Wl,-Map=main.map,--cref
avr-objcopy -O ihex -R .eeprom main.elf main.hex
avr-size -A -d main.elf
```
- Result HEX file will be `main.hex`
- You can then flash the code to the board using `avrdude -v -V -patmega328p -carduino "-PCOM10" -b115200 -D "-Uflash:w:main.hex:i"`. Replace `COM10` with your `COM` port (on Windows), or `/dev/ttyUSB*` port on Linux. (Idk about Mac, but it should be the same)

> [!IMPORTANT]  
> If you build for another AVR microcontroller, or if you use different clock speed, please change the `-mmcu=atmega328p` and `-DF_CPU=16000000UL` options to match your situation.

> [!NOTE]  
> Windows users can refer to the script `build.bat` in this repo. Just change the path to your correct path and it will work.

### Usage
After completing 2 sections above, you are now ready to boot Linux on Arduino UNO. Just connect the SD card to Arduino UNO, insert the card in, then power the Arduino on. Open a serial connection at baudrate 9600 bps to connect.
<br> You should see the following output (immidiately):
```text
arv32-opt: mini-rv32ima on Arduino UNO
SD card initialized successfully!
```

and (optionally, but normally, if your board is fast enough) the line `Currently at 0xB8` after these 2 lines.

> [!IMPORTANT]  
> The `Preparing the SD card` section must be repeated every time you boot your emulator. Otherwise it might not boot (if Linux has initialized enough and start cleaning the memory). This might be fixed in the future.

> [!IMPORTANT]  
> If you get `Error initializaing SD card` when you just plugged your board to your computer, just press the reset button. It should work.

You can also dump the state of the emulator while it's running. Just connect a button to `GND` and then connect pin 9 to the button. When you click the button, state will be dump, and you should see something like this:
```text
Effective emulated speed: 191 Hz, dtime=3299929ms, dcycle=631808
==============================================================================
Dumping emulator state:
Registers x0 - x31:
0x00000000 0x80032D4C 0x8027FC70 0x802C29D0
0x80281340 0x000A6465 0x00006C62 0x64656C62
0x8027FD30 0x802BD0E8 0x80284E0C 0x00000003
0x00000000 0x8027FC94 0x00000001 0x00000400
0x80284E0C 0x0000FFFF 0x00000000 0x00000000
0x802C6FEC 0x00000000 0x802C3000 0x8027FD4B
0x00000000 0x802C7000 0xFFFFFFFF 0x00000000
0x802C918C 0x802C918C 0x802C916C 0x8027FCCC
pc: 0x80035718
mstatus: 0x00000000
cyclel: 0x0009A400
cycleh: 0x00000000
timerl: 0x0009A000
timerh: 0x00000000
timermatchl: 0x00000000
timermatchh: 0x00000000
mscratch: 0x00000000
mtvec: 0x80001B78
mie: 0x00000000
mip: 0x00000000
mepc: 0x00000000
mtval: 0x00000000
mcause: 0x00000000
extraflags: 0x015873A3
==============================================================================

Dump completed. Emulator will continue when B1 is set back to HIGH
```
`Effective emulated speed` is the number of instructions the emulator can execute in 1 second at that time. `dtime` is the time difference between current time and the last time you dump the status. `dcycle` is the number of cycle (instructions) excuted from the last time you dump the status until now.
<br>`Registers x0 - x31` sections show registers from `x0` to `x31`, listed in order from left to right then from top down.

> [!NOTE]  
> The emulator will resume when you release the button (you should see something like `B1 is set to HIGH, emulator resume`). As long as you keep holding the button, the emulator will pause. You can also use a wire connected to GND instead of a button (just like me).

## Things can be changed in the code
- UART baudrate @ `main.c`
- SPI bus speed @ `main.c`
- Dump state trigger button @ `main.c`
- SD's CS pin @ `spi.h`

## Current issues and drawbacks
- The effective emulated speed is super slow. This is mostly due to the high overhead of 1-bit SPI connection with SD card, when every instruction that access memory must also access SD card.
- Max tested SPI bus speed is FCLK/16, which is 1MHz on 16MHz atmega328p. If you set the speed higher than this, SD card won't initialize. This might be just my problem, so feel free to try.
- Automatically replacing new state is not implemented yet. So every time you run the emulator, you must repeat the `Preparing the SD card` section.

## Credits
- [cnlohr](https://github.com/cnlohr) for writing [mini-rv32ima](https://github.com/cnlohr/mini-rv32ima/).
- [ryanj1234](https://github.com/ryanj1234) for writing [SD_TUTORIAL_PART4](https://github.com/ryanj1234/SD_TUTORIAL_PART4).
- [adnbr](https://github.com/adnbr/) for writing [1 ms counter](https://gist.github.com/adnbr/2439125)
- [me (gvl610/raspiduino)](https://github.com/raspiduino) for bringing all this stuff together.

## One last thing
If you can run this, you probably are running world's worst Linux PC. Enjoy!
