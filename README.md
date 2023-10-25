# arv32-opt
[Tested successfully!] Atmega328p port of mini-rv32ima. Let's run Linux on ~~the world's worst~~ one of the world's worst Linux PC (and beat Dmitry Grinberg)

![image](https://github.com/raspiduino/arv32-opt/assets/68118236/e457c5f7-110e-457c-ab98-48de47102af9)

[Video on Youtube](https://youtu.be/ZzReAELagG4)

Note: The code is in **pure AVR C**. Arduino IDE is just used as *serial terminal*.

UPDATE: I found a worse Linux PC: [C64](https://github.com/onnokort/semu-c64)!

## What is this?
This is a port of [mini-rv32ima](https://github.com/cnlohr/mini-rv32ima) (a minimum RISC-V emulator, capable of booting Linux) on atmega328p (the core of Arduino UNO, a 8-bit AVR microcontroller). So basically, this code is for **booting Linux on Arduino UNO**.

Yes you are reading it correctly, Arduino UNO can (theorically, but not practically) boot Linux. And it definitely beats [Dmitry Grinberg's (once) world's worst Linux PC](https://dmitry.gr/?r=05.Projects&proj=07.%20Linux%20on%208bit).

As seen on

## How does it work?
The idea is really simple: you have an Arduino UNO (or atmega328p) to run the emulator's logic, and emulator's RAM is accessed via swapping with an SD card (which is communicated through SPI interface, see more below). The emulator also has 3 512-bytes cache (1 icache and 2 dcache interchangable) and lazy/delayed cache write system.

The code is written in pure C (and not Arduino) to reduce Arduino overhead (if any). It initializes UART, SPI, SD card, and a digital input-pullup pin for triggering emulator state dump. Finally, it initialize cache, then mini-rv32ima and let the emulator does its works.

## How fast is it?
About ~~175Hz - 205Hz~~ ~~426 - 600Hz~~ most of the time ~~700 Hz~~ 1100Hz, peak ~~1500Hz~~ 2000Hz, lowest 70Hz with `-O3` code on an Arduino UNO based on atmega328p, clocked at 16MHz, with a class 4 SDHC card connected via 1-bit SPI interface. **[WARNING: NOT YET UPDATED]** Complete boot time (from start to shell) is about 15 hours and 44 minutes.

Update 24/9/2023: The speed is double/tripled by implementing icache

Update 26/9/2023: The speed is x1.5 by implementing 3 cache + lazy write system

Update 25/10/2023: SPI speed is x8, thanks to @kittennbfive's [suggestion](https://github.com/raspiduino/arv32-opt/issues/4)

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
> You should backup all your files, since doing this SD card preparation will destroy your SD card's filesystem.
- Then you need some tool to directly write a file to SD card. On Windows, you can use [HDD Raw Copy Tool](https://hddguru.com/software/HDD-Raw-Copy-Tool/), on *nix you can use `dd if=file.bin of=/dev/sdX conv=notrunc`.
- Download the file [`arv32.bin`](https://github.com/raspiduino/arv32-opt/raw/main/rv32.bin) file.
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

> [!IMPORTANT]  
> The `Preparing the SD card` section must be repeated every time you boot your emulator. Otherwise it might not boot (if Linux has initialized enough and start cleaning the memory). This might be fixed in the future.

> [!IMPORTANT]  
> If you get `Error initializaing SD card` when you just plugged your board to your computer, just press the reset button. It should work.

You can also dump the state of the emulator while it's running. Just connect a button to `GND` and then connect pin 9 to the button. When you click the button, state will be dump, and you should see something like this:
```text
Effective emulated speed: 1442 Hz, dtime=233596ms, dcycle=336896
Current AVR free memory: 155 bytes
icache hit/miss: 17931786/983542; dcache hit/miss: 4078057/1731330
==============================================================================
Dumping emulator state:
Registers x0 - x31:
0x00000000 0x800E6B34 0x8041F7A0 0x80332FC8
0x80420000 0x9B779A7E 0xDFB0F4A6 0x29EEBE42
0x8041F890 0xDEC0D0A6 0x9EE3B78E 0xA125AEE8
0x0A29015A 0x969C9575 0x31D9B5BA 0x1A086279
0xEFA9A208 0xAF715DDD 0x10C7F938 0x42557158
0x9A2FA369 0x6AA2616B 0x8041F8B4 0x00000000
0x00000000 0x36AAA20D 0x4BD078AB 0x000AF715
0x68A94F06 0x53A31796 0xF822BFDD 0x1A73C5BB
pc: 0x800E7144
mstatus: 0x00001880
cyclel: 0x0120A000
cycleh: 0x00000000
timerl: 0x00904E00
timerh: 0x00000000
timermatchl: 0x009050BF
timermatchh: 0x00000000
mscratch: 0x00000000
mtvec: 0x80001CBC
mie: 0x00000088
mip: 0x00000000
mepc: 0x800E6640
mtval: 0x00000000
mcause: 0x80000007
extraflags: 0x019446E3
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

## As seen on
[Hackaday](https://hackaday.com/2023/10/13/because-you-can-linux-on-an-arduino-uno/), [Hackster](https://www.hackster.io/news/giang-vinh-loc-creates-the-world-s-worst-linux-pc-using-an-arduino-uno-r3-and-its-atmega328p-e5ed03e3f594), [Habr](https://habr.com/ru/news/767550/), [Maker News](https://news.mkme.org/?p=66623), [internetua](https://internetua.com/entuziast-zapustiv-linux-na-arduino-uno), [futuranet.it](https://ei.futuranet.it/2023/10/24/arduino-uno-linux-su-atmega328/), [zhihu.com](https://zhuanlan.zhihu.com/p/662411944)

## One last thing
If you can run this, you probably are running world's worst Linux PC. Enjoy!
