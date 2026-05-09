# avr-mod-player

An ATmega32-based ProTracker MOD music player with AM29F040B parallel flash memory and stereo PWM audio output. Program a MOD file onto the flash chip, plug it in, and music starts playing.

![AVR MOD Player](/docs/images/avr-mod-player-1.jpg)

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Getting Started](#getting-started)
  - [What You Need](#what-you-need)
  - [Programming the Flash Chip](#programming-the-flash-chip)
  - [Building and Flashing the Firmware](#building-and-flashing-the-firmware)
- [Schematic](#schematic)
- [How It Works](#how-it-works)
  - [Flash Memory Interface](#flash-memory-interface)
  - [Audio Output](#audio-output)
  - [MOD Playback Engine](#mod-playback-engine)
  - [Channel Activity LEDs](#channel-activity-leds)
  - [Power Supply and Protection](#power-supply-and-protection)
- [Speaker Extension Board](#speaker-extension-board)
- [Documentation](#documentation)
- [Acknowledgments](#acknowledgments)

---

## Overview

This is a standalone hardware MOD file player built around the ATmega32A microcontroller. It reads ProTracker .MOD files directly from an AM29F040B parallel NOR flash chip and outputs stereo audio through RCA connectors using PWM and analog filtering. Four green LEDs flash in time with the music, one per channel, showing which voices are active.

![AVR MOD Player Top View](/docs/images/avr-mod-player-2.jpg)

The concept is simple: program a MOD file onto the flash chip, plug it into the socket, flip the power switch, and music starts playing immediately. No SD cards, no file systems, no menus. Just a chip full of music and a microcontroller that knows how to play it. If you want to listen to a different track, pull the flash chip out of its socket, erase it, program a new file, and put it back.

My main inspiration was [MadWizard's AVR MOD player](https://www.madwizard.org/electronics/projects/modplayer). That project also used the AM29F040 flash chip, which led me down this path. Since I had no way to program these chips, I ended up designing my own flash programmer, available as a separate project: [amd-flash-programmer](https://github.com/bazsimarkus/amd-flash-programmer).

---

## Features

- ATmega32A microcontroller running at 16 MHz external crystal
- AM29F040B 512KB parallel NOR flash for MOD file storage (socketed for easy swapping)
- Stereo audio output through dual RCA connectors
- MCP6022 dual rail-to-rail op-amp based analog output filtering
- 74HC595 shift register driving four green 3mm channel activity LEDs
- 4-channel ProTracker MOD format playback with effects (arpeggio, portamento, vibrato, volume slide, and more)
- 16 kHz PWM sample rate via Timer1, stereo hard-panned per ProTracker convention (channels 1 and 4 left, channels 2 and 3 right)
- 10-pin ISP header for in-circuit programming via USBasp
- 9-12V DC barrel jack power input with L7805 5V regulator
- 1N5819 Schottky diode protection preventing backfeed between DC jack and ISP programmer power
- ON/OFF toggle switch and reset button
- Pin header for the optional LM386 speaker extension board
- Designed in KiCad, full schematic and PCB files included

---

## Getting Started

### What You Need

- The assembled AVR MOD Player board
- A USBasp programmer (or any AVR ISP programmer)
- A way to program the AM29F040B flash chip (e.g. [amd-flash-programmer](https://github.com/bazsimarkus/amd-flash-programmer))
- A ProTracker .MOD file (max 512KB to fit on the flash chip)
- avr-gcc and avrdude installed on your system
- An RCA audio cable or the speaker extension board for output

### Programming the Flash Chip

1. Find a MOD file you want to play. [The Mod Archive](https://modarchive.org/) has a large collection of free tracks. Some personal favorites: Maktone - Vanishing Colors, LHS - It's a Good Day, Random Voice - Monday.

2. Remove the AM29F040B from its socket on the player board.

3. Program the raw .MOD file onto the flash chip starting at address 0x00000 using the [amd-flash-programmer](https://github.com/bazsimarkus/amd-flash-programmer) or any compatible flash programmer.

4. Place the programmed flash chip back into the socket on the player board.

5. Flip the power switch. The music starts playing immediately.

### Building and Flashing the Firmware

The firmware is a single C source file (`avr-mod-player/main.c`) targeting the ATmega32A at 16 MHz. It has no external dependencies beyond standard avr-libc. It links against libm for the math routines.

#### Method 1: Command line with avr-gcc and avrdude

Install avr-gcc and avrdude for your platform (on Debian/Ubuntu: `sudo apt install gcc-avr avr-libc avrdude`; on Windows, install WinAVR or the Microchip AVR toolchain).

Navigate to the `avr-mod-player` directory and run:

```sh
avr-gcc -mmcu=atmega32 -DF_CPU=16000000UL -O1 \
    -funsigned-char -funsigned-bitfields \
    -ffunction-sections -fdata-sections \
    -fpack-struct -fshort-enums \
    -Wall -o main.elf main.c -lm

avr-objcopy -O ihex main.elf main.hex
```

Then flash with your USBasp programmer:

```sh
avrdude -c usbasp -p m32 -U flash:w:main.hex
```

#### Method 2: MPLAB X IDE

The repository includes an MPLAB X IDE project in the `avr-mod-player/nbproject` directory. The project has two build configurations:

- **default**: Compiles only, produces the hex file.
- **flash**: Identical to default, but runs avrdude automatically after a successful build to flash the connected board via USBasp. The post-build step is:

```
avrdude -c usbasp -p m32 -U flash:w:${ImagePath}
```

With the flash configuration selected, clicking Build Main Project compiles the firmware and programs the board in a single step.

The project was developed with MPLAB X IDE v6.25 and the Microchip ATmega_DFP 3.3.279 device pack. The compiler is avr-gcc 5.4.0.

---

## Schematic

![AVR MOD Player Schematic](/docs/images/avr-mod-player-schematic.jpg)

The full KiCad project files (schematic, PCB layout, and project file) are in the `schematic/` directory.

---

## How It Works

### Flash Memory Interface

The AM29F040B is a 512KB parallel NOR flash chip in a 32-pin DIP package. The ATmega32A addresses it using nearly all of its available I/O pins:

- PORTA (PA0-PA7): Address lines A0-A7
- PORTB (PB0-PB7): Address lines A8-A15
- PORTD (PD0-PD2): Address lines A16-A18
- PORTC (PC0-PC7): Data bus DQ0-DQ7 (configured as input)

This gives 19 address lines, enough for the full 512KB address space. The chip enable (CE#) and output enable (OE#) pins are active (directly connected to ground), meaning the flash is always outputting data on the bus. The write enable (WE#) pin is pulled high permanently with a resistor, since the player board never writes to the flash. This simplified design was necessary because the ATmega32A does not have enough free pins to control WE# dynamically.

Reading a byte from flash is a single function call: set PORTA, PORTB, and the lower bits of PORTD to the desired address, wait two CPU cycles for the flash access time, then read PINC. At 16 MHz, this gives about 187 ns of access time, well within the 90 ns rating of the AM29F040B-90PC.

### Audio Output

Stereo audio is generated using Timer1 in 8-bit fast PWM mode. The two PWM outputs (OC1A on PD5 for left, OC1B on PD4 for right) run at a carrier frequency of 62.5 kHz (16 MHz / 256), which is well above the audible range.

The raw PWM signals pass through a low-pass filter built around the MCP6022 dual rail-to-rail op-amp. Each channel has its own filter section: a 10nF capacitor and a 4.7k resistor form the RC low-pass, and the op-amp buffers and drives the output through a 220uF coupling capacitor to the RCA connector. The coupling capacitor removes the DC offset so only the audio AC signal reaches the amplifier or speakers.

### MOD Playback Engine

The firmware implements a ProTracker-compatible 4-channel MOD player. Timer0 fires an interrupt at exactly 16,000 Hz, which pulls samples from a ring buffer and writes them to the PWM registers. The main loop continuously fills that buffer by mixing the four channels.

The MOD file format stores music as a sequence of patterns. Each pattern has 64 rows, and each row has four cells (one per channel). Each cell can trigger a sample at a given pitch, change the volume, or apply an effect. At startup, the firmware parses the MOD header from flash to locate the sample data and pattern data, then begins sequencing through the song.

The engine supports the following ProTracker effects:

- 0xy: Arpeggio
- 1xx: Portamento up
- 2xx: Portamento down
- 3xx: Tone portamento
- 4xy: Vibrato
- 9xx: Sample offset
- Axy: Volume slide
- Bxx: Pattern jump
- Cxx: Set volume
- Dxx: Pattern break
- Fxx: Set speed/tempo
- E1y/E2y: Fine portamento
- E9y: Retrigger note
- EAy/EBy: Fine volume slide
- ECy: Note cut

Pitch is calculated using 16.16 fixed-point arithmetic. The playback step for each channel is derived from the Amiga PAL clock constant (3,546,895 Hz) divided by the channel's current period value, scaled to the 16 kHz output rate. This gives sample-accurate pitch reproduction matching the original Amiga hardware.

The MOD file format specification used to build this player is included in the repository at `docs/mod-spec.txt` (sourced from [eblong.com/zarf/blorb/mod-spec.txt](https://www.eblong.com/zarf/blorb/mod-spec.txt)).

### Channel Activity LEDs

Four green 3mm LEDs (D1-D4) are driven by a 74HC595 shift register connected to three pins on PORTD. Each time a note triggers on a channel, the corresponding LED lights up for approximately 60 milliseconds (3 sequencer ticks). This creates a short flash locked to the beat, giving a visual indication of which channels are active. The LEDs are updated once per sequencer tick in the main audio loop.

### Power Supply and Protection

The board is powered from a 9-12V DC barrel jack. An L7805 voltage regulator provides the 5V rail for all digital and analog circuitry. A 1N4001 rectifier diode on the DC input provides reverse polarity protection.

A 1N5819 Schottky diode sits between the ISP header's VCC pin and the board's 5V rail. When the board is powered from the DC jack, the Schottky diode prevents the regulated 5V from backfeeding into the programmer. When the board is powered from the programmer (for firmware development without the DC supply connected), the Schottky diode conducts and powers the board from the programmer's VCC. This protects both the L7805 regulator and the programmer from conflicting voltage sources.

---

## Speaker Extension Board

![Speaker Extension Board](/docs/images/avr-mod-player-speaker-board-1.jpg)

An optional LM386 speaker extension board can be connected to the 2x4 pin header (J4) on the main board. It mixes the left and right channels through two 300k resistors into a volume trimmer, feeds the signal through an LM386 audio power amplifier, and drives a small 23mm 8-ohm speaker. The board connects directly to the pin header and receives both audio signals and power from the main board.

![Speaker Extension Board connected to the main board](/docs/images/avr-mod-player-speaker-board-2.jpg)

The KiCad project files for the speaker board are in `extension-boards/speaker-board/`.

---

## Documentation

The `docs/` directory contains:

- `mod-spec.txt`: The ProTracker MOD file format specification used as reference for the playback engine (from [eblong.com](https://www.eblong.com/zarf/blorb/mod-spec.txt))

---

## Acknowledgments

- [MadWizard](https://www.madwizard.org/electronics/projects/modplayer) for the original AVR MOD player project that inspired this build
- [The Mod Archive](https://modarchive.org/) for the collection of freely available MOD files
- The ProTracker MOD format specification from [eblong.com](https://www.eblong.com/zarf/blorb/mod-spec.txt)
