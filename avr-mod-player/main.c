/*
 * main.c ? ATmega32A MOD Player (ProTracker-compatible, 4-channel)
 *
 * Hardware:
 *   - AM29F040B parallel NOR flash (512KB) hard-wired to PORTA (A0-A7),
 *     PORTB (A8-A15), and PD0..PD2 (A16-A18). PORTC = data bus (input).
 *   - Stereo PWM audio output via Timer1: OC1A (PD5) = Left, OC1B (PD4) = Right.
 *   - 74HC595 shift register for 4-channel LED visualisation via PD3/PD6/PD7.
 *   - Timer0 fires the audio ISR at exactly 16,000 Hz.
 *
 * Converted from Arduino sketch to bare-metal avr-gcc / MPLAB X.
 * Minimal changes: setup() -> init code at top of main(), loop() -> while(1).
 * All functionality is identical to the original Arduino version.
 *
 * Toolchain: MPLAB X IDE + avr-gcc (or XC8 in C99 mode)
 * Target:    ATmega32A @ 16 MHz external crystal
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// =========================================================================
// DEFINES & CONSTANTS
// =========================================================================

#define SAMPLE_RATE      16000
#define AMIGA_PAL_CLOCK  3546895UL

// 16.16 fixed-point constant for playback step calculation.
// step = STEP_CONSTANT / period  =>  gives the number of sample-data bytes
// to advance per output sample at 16 kHz, in 16.16 fixed-point.
// Derivation: (AMIGA_PAL_CLOCK * 65536) / SAMPLE_RATE = 14528082
#define STEP_CONSTANT  14528082UL

#define NUM_SAMPLES       31
#define CHANNELS           4
#define ROWS_PER_PATTERN  64
#define BUF_SIZE         128   // Must be a power of two (ring-buffer)

// Shift Register Pins (all on PORTD)
#define SER_PIN    PD3   // Serial data in
#define SRCLK_PIN  PD6   // Shift register clock
#define RCLK_PIN   PD7   // Latch / storage clock

// =========================================================================
// PROGMEM TABLES
// =========================================================================

// Sine table for vibrato effect (effect 0x4).
// Range: -127 .. +127, 64 entries covering one full cycle.
static const int8_t sine_table[64] PROGMEM = {
      0,  12,  25,  37,  49,  60,  71,  81,  90,  98, 106, 112, 117, 122, 125, 126,
    127, 126, 125, 122, 117, 112, 106,  98,  90,  81,  71,  60,  49,  37,  25,  12,
      0, -12, -25, -37, -49, -60, -71, -81, -90, -98,-106,-112,-117,-122,-125,-126,
   -127,-126,-125,-122,-117,-112,-106, -98, -90, -81, -71, -60, -49, -37, -25, -12
};

// Pre-computed 16.16 fixed-point multipliers for arpeggio (effect 0x0).
// Each entry approximates pow(2, -semitone/12) * 65536.
// Index 0 = unison (65536 ? 1.0), index 12 = one octave down (32768 = 0.5), etc.
// Usage: new_period = (period * arp_factor[semitones]) >> 16
static const uint16_t arp_factor[16] PROGMEM = {
    65535, 61858, 58386, 55109, 52016, 49097, 46341, 43740,
    41285, 38968, 36781, 34716, 32768, 30929, 29193, 27554
};

// =========================================================================
// DATA STRUCTURES
// =========================================================================

// Describes one of the 31 sample slots stored in the MOD file header.
typedef struct {
    uint32_t data_offset;   // Absolute byte address of sample PCM data in flash
    uint32_t length;        // Total sample length in bytes
    uint32_t loop_start;    // Loop start offset in bytes
    uint32_t loop_length;   // Loop region length in bytes (<=2 means no loop)
    uint8_t  volume;        // Default volume (0-64)
} Sample;

// Global module metadata, parsed once at startup from flash.
typedef struct {
    Sample   samples[NUM_SAMPLES];
    uint8_t  num_song_positions;   // How many entries in the order table are valid
    uint8_t  restart_position;     // Song restart position (often unused)
    uint8_t  order[128];           // Pattern play order table
    uint8_t  num_patterns;         // Total number of unique patterns
    uint32_t pattern_data_offset;  // Byte address in flash where pattern data begins
} Module;

// Per-channel runtime state for the audio engine.
typedef struct {
    // Sample data (copied from Sample struct when a note triggers)
    uint32_t data_offset;
    uint32_t length;
    uint32_t loop_start;
    uint32_t loop_length;
    uint8_t  volume;        // Current volume (0-64), may be altered by effects

    uint32_t position;      // Current playback position in sample, 16.16 fixed-point
    uint32_t step;          // Playback increment per output sample, 16.16 fixed-point
    uint16_t period;        // Current Amiga period value (controls pitch). 0 = silent.

    // Effect memory (persists across ticks / rows)
    uint16_t porta_target;  // Tone portamento (0x3) destination period
    uint8_t  porta_speed;   // Tone portamento slide speed
    uint8_t  vib_speed;     // Vibrato (0x4) speed
    uint8_t  vib_depth;     // Vibrato depth
    uint8_t  vib_pos;       // Vibrato sine table position (0-63)
    uint8_t  arp_param;     // Arpeggio (0x0) high/low nibble semitone param

    uint8_t  triggered;     // Set to 1 on tick 0 when a new note fires (for LEDs)
} Channel;

// Global sequencer / timing state.
typedef struct {
    uint8_t  song_pos;          // Current position in the order table
    uint8_t  pattern_row;       // Current row within the pattern (0-63)
    uint8_t  tick;              // Current tick within the row (0 .. ticks_per_row-1)
    uint8_t  ticks_per_row;     // Speed (default 6); set by effect 0xF with param < 0x20
    uint8_t  bpm;               // Tempo in BPM (default 125); set by effect 0xF param >= 0x20
    int16_t  samples_per_tick;  // How many audio samples make up one sequencer tick
    int16_t  samples_remaining; // Countdown until the next process_tick() call
    Channel  channel[CHANNELS];
} Player;

// One decoded cell (note slot) from the pattern data.
typedef struct {
    uint8_t  sample;  // Sample number (1-31), or 0 if no new sample
    uint16_t period;  // Amiga period (pitch), or 0 if no new note
    uint8_t  effect;  // Effect number (0x0 - 0xF)
    uint8_t  param;   // Effect parameter byte
} Cell;

// =========================================================================
// GLOBALS
// =========================================================================

Module mod;
Player player;

// Double-buffered ring buffers for left and right PWM audio (filled by main loop,
// consumed by the Timer0 ISR at 16 kHz).
uint8_t audio_buf_L[BUF_SIZE];
uint8_t audio_buf_R[BUF_SIZE];
volatile uint8_t rd_ptr = 0;  // Read pointer ? advanced by ISR
uint8_t wr_ptr = 0;           // Write pointer ? advanced by main loop
volatile uint8_t audio_running = 0;

// =========================================================================
// HARDWARE FLASH ABSTRACTION
// =========================================================================

// Reads a single byte from the hardwired AM29F040B flash at 'addr'.
// PORTA = A0-A7, PORTB = A8-A15, PD0-PD2 = A16-A18, PORTC = data (input).
// The two NOPs after writing PORTD give the flash its required address
// setup + access time (max 150 ns; 2 cycles @ 16 MHz ? 125 ns + PORTD write ? 187.5 ns total).
inline uint8_t read_flash(uint32_t addr) {
    PORTA = addr & 0xFF;
    PORTB = (addr >> 8) & 0xFF;
    // Preserve PWM (PD4/PD5) and 74HC595 (PD3/PD6/PD7) pins; only touch PD0-PD2
    PORTD = (PORTD & 0xF8) | ((addr >> 16) & 0x07);

    // Wait for address propagation & flash access time (up to 150ns)
    // At 16MHz, setting PORTD takes 1 cycle, plus 2 nops = ~187.5ns total.
    __asm__ __volatile__ ("nop\n\t" "nop\n\t");

    return PINC;
}

// Reads a big-endian 16-bit word from flash (MOD format is big-endian).
uint16_t read_be16(uint32_t addr) {
    uint8_t hi = read_flash(addr);
    uint8_t lo = read_flash(addr + 1);
    return (hi << 8) | lo;
}

// =========================================================================
// ENGINE ROUTINES
// =========================================================================

// Recalculates samples_per_tick from the current BPM.
// ProTracker formula: samples_per_tick = (SAMPLE_RATE * 5) / (BPM * 2)
//                                      = 40000 / BPM
void update_tick_timing() {
    player.samples_per_tick = 40000 / player.bpm;
}

// Triggers a new note on a channel: resets playback position and
// recalculates the step size from the current period value.
void trigger_note(Channel* ch) {
    ch->position  = 0;
    ch->triggered = 1;
    if (ch->period > 0) {
        ch->step = STEP_CONSTANT / ch->period;
    }
}

// -------------------------------------------------------------------------
// process_effect_tick0()  ? effects that apply only on tick 0 (note onset)
// -------------------------------------------------------------------------
// MOD effect command reference (all are 4-bit hex nibbles, 0x0 .. 0xF):
//
//  0x0 pp  ? Arpeggio: on tick 0, just store param. Actual pitch-cycling
//             happens in process_effect_tick_n().  pp = 0 resets arpeggio.
//  0x3 pp  ? Tone Portamento: store speed and target period. Does NOT
//             trigger a new note ? the pitch slides from the current period
//             to the target. Tick-by-tick sliding happens in tick_n.
//  0x9 pp  ? Sample Offset: start playback pp*256 bytes into the sample.
//  0xB pp  ? Pattern Jump: jump to order-table position pp immediately.
//  0xC pp  ? Set Volume: set channel volume to pp (0-64, clamped).
//  0xD pp  ? Pattern Break: advance to the next pattern, start at row pp
//             (pp is BCD: high nibble*10 + low nibble).
//  0xF pp  ? Set Speed/Tempo: pp < 0x20 sets ticks-per-row (speed);
//             pp >= 0x20 sets BPM.
//  0xE 1y  ? Fine Portamento Up:   raise period by y immediately (tick 0 only).
//  0xE 2y  ? Fine Portamento Down: lower period by y immediately (tick 0 only).
//  0xE Ay  ? Fine Volume Slide Up:   add y to volume (tick 0 only).
//  0xE By  ? Fine Volume Slide Down: subtract y from volume (tick 0 only).
void process_effect_tick0(Channel* ch, Cell* cell) {
    uint8_t eff   = cell->effect;
    uint8_t param = cell->param;
    uint8_t px    = (param >> 4) & 0x0F;  // High nibble of param
    uint8_t py    = param & 0x0F;          // Low nibble of param

    switch (eff) {
        case 0x0:
            // Arpeggio ? store the param; tick_n handles the cycling.
            // param == 0 means "no arpeggio" (normal note).
            if (param != 0) ch->arp_param = param;
            break;

        case 0x3:
            // Tone Portamento ? remember speed and target, but do NOT
            // retrigger the note (handled specially in process_tick()).
            if (param != 0) ch->porta_speed  = param;
            if (cell->period != 0) ch->porta_target = cell->period;
            break;

        case 0x9:
            // Sample Offset ? jump playback head to param * 256 bytes.
            // Stored as 16.16 fixed-point (shift left 16).
            if (param != 0) ch->position = (uint32_t)(param * 256) << 16;
            break;

        case 0xB:
            // Pattern Jump ? set song position; row rolls over to 0 via the
            // pattern_row = 255 trick (255 + 1 wraps to 0 after the tick++ below).
            player.song_pos = param;
            if (player.song_pos >= mod.num_song_positions) player.song_pos = 0;
            player.pattern_row = 255; // Rollover handled in tick++
            break;

        case 0xC:
            // Set Volume ? clamp to 64.
            ch->volume = (param > 64) ? 64 : param;
            break;

        case 0xD:
            // Pattern Break ? advance to next pattern, start at BCD row pp.
            player.song_pos++;
            if (player.song_pos >= mod.num_song_positions) player.song_pos = 0;
            player.pattern_row = (px * 10 + py) - 1; // -1 because tick++ will add 1
            break;

        case 0xF:
            // Set Speed / Tempo
            if (param < 0x20) {
                // param 1..31 = ticks per row (sequence speed)
                if (param > 0) player.ticks_per_row = param;
            } else {
                // param 32..255 = BPM
                player.bpm = param;
                update_tick_timing();
            }
            break;

        case 0xE:
            // Extended effects (Exy ? x = sub-command, y = value)
            switch (px) {
                case 0x1:
                    // E1y ? Fine Portamento Up: decrease period by y (raises pitch).
                    // Period is inversely proportional to frequency.
                    if (ch->period > 0) {
                        ch->period -= py;
                        if (ch->period < 113) ch->period = 113; // Clamp to highest MOD pitch
                        ch->step = STEP_CONSTANT / ch->period;
                    }
                    break;

                case 0x2:
                    // E2y ? Fine Portamento Down: increase period by y (lowers pitch).
                    if (ch->period > 0) {
                        ch->period += py;
                        if (ch->period > 856) ch->period = 856; // Clamp to lowest MOD pitch
                        ch->step = STEP_CONSTANT / ch->period;
                    }
                    break;

                case 0xA:
                    // EAy ? Fine Volume Slide Up: add y, clamp to 64.
                    ch->volume += py;
                    if (ch->volume > 64) ch->volume = 64;
                    break;

                case 0xB:
                    // EBy ? Fine Volume Slide Down: subtract y, clamp to 0.
                    if (ch->volume >= py) ch->volume -= py; else ch->volume = 0;
                    break;
            }
            break;
    }
}

// -------------------------------------------------------------------------
// process_effect_tick_n() ? effects that apply on ticks 1..ticks_per_row-1
// -------------------------------------------------------------------------
// Effects that continuously modulate pitch or volume every tick:
//
//  0x0 pp  ? Arpeggio: cycle through base, +x semitones, +y semitones.
//             tick%3==0 => base period, 1 => +px semitones, 2 => +py semitones.
//  0x1 pp  ? Portamento Up: decrease period by pp each tick (raises pitch).
//  0x2 pp  ? Portamento Down: increase period by pp each tick (lowers pitch).
//  0x3 pp  ? Tone Portamento: slide period toward porta_target by porta_speed.
//  0x4 Xd  ? Vibrato: sinusoidal pitch wobble. X = speed, d = depth.
//             Uses sine_table[]; depth is in units of 1/64 of a period step.
//  0xA pp  ? Volume Slide: px = slide up amount, py = slide down amount.
//             Only one of px/py should be nonzero (if both set, px wins).
//  0xE 9y  ? Retrigger Note: retrigger every y ticks.
//  0xE Cy  ? Note Cut: set volume to 0 at tick y.
void process_effect_tick_n(Channel* ch, Cell* cell, uint8_t tick) {
    uint8_t eff   = cell->effect;
    uint8_t param = cell->param;
    uint8_t px    = (param >> 4) & 0x0F;  // High nibble
    uint8_t py    = param & 0x0F;          // Low nibble

    switch (eff) {
        case 0x0:
            // Arpeggio ? three-note cycling: base / +px semitones / +py semitones.
            if (ch->arp_param != 0 && ch->period != 0) {
                uint8_t semi = 0;
                if (tick % 3 == 1) semi = (ch->arp_param >> 4) & 0xF;  // px semitones
                if (tick % 3 == 2) semi = ch->arp_param & 0xF;          // py semitones
                // Scale period down by arp_factor[semi]/65536 to raise pitch by 'semi' semitones
                uint32_t p = ch->period;
                p = (p * pgm_read_word(&arp_factor[semi])) >> 16;
                ch->step = STEP_CONSTANT / p;
            }
            break;

        case 0x1:
            // Portamento Up ? decrease period by param each tick (raises pitch).
            if (param != 0 && ch->period != 0) {
                if (ch->period > param) ch->period -= param; else ch->period = 1;
                if (ch->period < 113) ch->period = 113;
                ch->step = STEP_CONSTANT / ch->period;
            }
            break;

        case 0x2:
            // Portamento Down ? increase period by param each tick (lowers pitch).
            if (param != 0 && ch->period != 0) {
                ch->period += param;
                if (ch->period > 856) ch->period = 856;
                ch->step = STEP_CONSTANT / ch->period;
            }
            break;

        case 0x3:
            // Tone Portamento ? slide current period toward porta_target by porta_speed.
            if (ch->porta_speed != 0 && ch->period != 0 && ch->porta_target != 0) {
                if (ch->period < ch->porta_target) {
                    // Slide up (period increases = pitch falls toward target)
                    ch->period += ch->porta_speed;
                    if (ch->period > ch->porta_target) ch->period = ch->porta_target;
                } else if (ch->period > ch->porta_target) {
                    // Slide down (period decreases = pitch rises toward target)
                    if (ch->period >= ch->porta_speed) ch->period -= ch->porta_speed;
                    else ch->period = ch->porta_target;
                    if (ch->period < ch->porta_target) ch->period = ch->porta_target;
                }
                ch->step = STEP_CONSTANT / ch->period;
            }
            break;

        case 0x4:
            // Vibrato ? sinusoidal pitch modulation using sine_table[].
            // px = speed (how fast vib_pos advances), py = depth.
            // delta = (depth * sine) / 64  =>  added to period before recalculating step.
            if (px != 0) ch->vib_speed = px;
            if (py != 0) ch->vib_depth = py;
            if (ch->period != 0) {
                int8_t  wave  = pgm_read_byte(&sine_table[ch->vib_pos & 63]);
                int16_t delta = ((int16_t)ch->vib_depth * wave) / 64;
                ch->step = STEP_CONSTANT / (ch->period + delta);
                ch->vib_pos = (ch->vib_pos + ch->vib_speed) & 63;
            }
            break;

        case 0xA:
            // Volume Slide ? px = slide up, py = slide down each tick.
            // If both nonzero, px (up) takes priority.
            {
                int8_t slide = 0;
                if      (px != 0 && py == 0) slide =  px;
                else if (py != 0 && px == 0) slide = -py;
                else if (px != 0 && py != 0) slide =  px; // px wins if both set
                int16_t new_vol = ch->volume + slide;
                if (new_vol > 64) new_vol = 64;
                if (new_vol < 0)  new_vol = 0;
                ch->volume = new_vol;
            }
            break;

        case 0xE:
            // Extended effects on non-zero ticks:
            if (px == 0x9 && py > 0 && (tick % py) == 0) {
                // E9y ? Retrigger Note: restart sample playback every y ticks.
                ch->position = 0;
            }
            if (px == 0xC && tick == py) {
                // ECy ? Note Cut: silence the channel at tick y (volume -> 0).
                ch->volume = 0;
            }
            break;
    }
}

// -------------------------------------------------------------------------
// process_tick() ? called once per sequencer tick by the main audio loop
// -------------------------------------------------------------------------
// On tick 0: read the next pattern row, decode 4 cells, load sample data,
//            apply tick-0 effects, and trigger notes.
// On tick N: re-read only the effect bytes and apply continuing effects.
void process_tick() {
    // Calculate the flash address of the current pattern row.
    // Pattern layout: pattern_index * 1024 bytes, each row = 4 channels * 4 bytes = 16 bytes.
    uint32_t row_addr = mod.pattern_data_offset
                      + (mod.order[player.song_pos] * 1024)
                      + (player.pattern_row * 16);

    if (player.tick == 0) {
        // --- Tick 0: decode full cell and trigger notes ---
        for (int ch = 0; ch < CHANNELS; ch++) {
            Channel* chn       = &player.channel[ch];
            uint32_t cell_addr = row_addr + ch * 4;

            // Each cell is 4 bytes packed as per ProTracker MOD format:
            //  Byte 0: [samp_hi(4)] [period_hi(4)]
            //  Byte 1: [period_lo(8)]
            //  Byte 2: [samp_lo(4)] [effect(4)]
            //  Byte 3: [param(8)]
            uint8_t r0 = read_flash(cell_addr);
            uint8_t r1 = read_flash(cell_addr + 1);
            uint8_t r2 = read_flash(cell_addr + 2);
            uint8_t r3 = read_flash(cell_addr + 3);

            Cell cell;
            cell.sample = (r0 & 0xF0) | ((r2 & 0xF0) >> 4); // Combine high and low nibbles
            cell.period  = ((uint16_t)(r0 & 0x0F) << 8) | r1;
            cell.effect  = r2 & 0x0F;
            cell.param   = r3;

            chn->triggered = 0; // Clear trigger flag; set again by trigger_note() if needed

            // If a sample number is present, load its metadata onto the channel.
            if (cell.sample >= 1 && cell.sample <= NUM_SAMPLES) {
                Sample* s      = &mod.samples[cell.sample - 1];
                chn->data_offset  = s->data_offset;
                chn->length       = s->length;
                chn->loop_start   = s->loop_start;
                chn->loop_length  = s->loop_length;
                chn->volume       = s->volume;
            }

            // Tone portamento (0x3) is special: we apply its tick-0 logic but
            // must NOT retrigger the note (the slide continues from the current pitch).
            if (cell.effect == 0x3) {
                process_effect_tick0(chn, &cell);
                continue; // Skip the normal trigger below
            }

            // Apply all other tick-0 effects first (e.g. 0xC sets volume before note plays).
            process_effect_tick0(chn, &cell);

            // If a period is given, update the channel pitch and trigger the note.
            if (cell.period != 0) {
                chn->period = cell.period;
                trigger_note(chn);
            }
        }
    } else {
        // --- Tick N (N > 0): apply only continuing/sustain effects ---
        for (int ch = 0; ch < CHANNELS; ch++) {
            Channel* chn       = &player.channel[ch];
            uint32_t cell_addr = row_addr + ch * 4;

            // Only the effect and param bytes are needed here; skip r0/r1.
            Cell cell;
            cell.effect = read_flash(cell_addr + 2) & 0x0F;
            cell.param  = read_flash(cell_addr + 3);
            process_effect_tick_n(chn, &cell, player.tick);
        }
    }

    // Advance tick counter, and move to the next row when the row is complete.
    player.tick++;
    if (player.tick >= player.ticks_per_row) {
        player.tick = 0;
        player.pattern_row++;
        if (player.pattern_row >= ROWS_PER_PATTERN) {
            player.pattern_row = 0;
            player.song_pos++;
            if (player.song_pos >= mod.num_song_positions) {
                player.song_pos = 0; // Loop song from beginning
            }
        }
    }
}

// =========================================================================
// LED VISUALIZATION (74HC595)
// =========================================================================

// Shifts 8 bits out to the 74HC595 shift register, MSB first.
// Each of the 4 lower bits controls an LED for one audio channel.
// LEDs are latched on for 3 sequence ticks (~60 ms) on each note trigger,
// giving a short, sharp flash locked to the beat onset.
void update_leds() {
    static uint8_t led_timers[CHANNELS] = {0, 0, 0, 0};
    uint8_t led_mask = 0;

    for (int ch = 0; ch < CHANNELS; ch++) {
        // If a new note was just triggered on this channel and it's not silent
        if (player.channel[ch].triggered && player.channel[ch].volume > 0) {
            // Set the LED to stay on for 3 sequence ticks (approx 60 milliseconds).
            // This creates a short, sharp flash exactly on the beat/note onset.
            led_timers[ch] = 3;
        }

        // Keep LED on while its timer is active, then decrement
        if (led_timers[ch] > 0) {
            led_mask |= (1 << ch);
            led_timers[ch]--;
        }
    }

    // Shift out 8 bits to the 74HC595 (MSB first)
    for (int i = 7; i >= 0; i--) {
        if (led_mask & (1 << i)) {
            PORTD |= (1 << SER_PIN);
        } else {
            PORTD &= ~(1 << SER_PIN);
        }

        // Pulse Shift Register Clock (rising edge shifts one bit in)
        PORTD |= (1 << SRCLK_PIN);
        PORTD &= ~(1 << SRCLK_PIN);
    }

    // Pulse Latch Clock to transfer shift register to output pins (LEDs update here)
    PORTD |= (1 << RCLK_PIN);
    PORTD &= ~(1 << RCLK_PIN);
}

// =========================================================================
// MODULE LOADER
// =========================================================================

// Parses the ProTracker MOD file header from flash and populates 'mod' and 'player'.
// MOD layout (M.K., 31-sample variant):
//   0x000          : 20 bytes ? song title (ignored)
//   0x014 .. 0x1D3: 31 * 30 bytes ? sample headers (name, length, finetune, vol, loop)
//   0x1D4         : 1 byte  ? number of song positions
//   0x1D5         : 1 byte  ? restart position (often 0x7F, unused here)
//   0x1D6 .. 0x255: 128 bytes ? pattern order table
//   0x256 .. 0x259: 4 bytes  ? signature ("M.K." or "4CHN" etc.)
//   0x25A .. end  : pattern data (num_patterns * 1024 bytes), then sample PCM data
void mod_load() {
    uint32_t addr = 20; // Skip 20-byte song title

    // --- Parse 31 sample headers ---
    for (int i = 0; i < NUM_SAMPLES; i++) {
        addr += 22; // Skip 22-byte sample name field
        uint16_t len       = read_be16(addr); addr += 2; // Length in words (multiply by 2 for bytes)
        addr++;                                            // Skip finetune byte (not implemented)
        mod.samples[i].volume = read_flash(addr++);       // Default volume (0-64)
        uint16_t rep_start = read_be16(addr); addr += 2;  // Loop start in words
        uint16_t rep_len   = read_be16(addr); addr += 2;  // Loop length in words (<=1 = no loop)

        // Convert word-addressed fields to byte addresses
        mod.samples[i].length      = (uint32_t)len       * 2;
        mod.samples[i].loop_start  = (uint32_t)rep_start * 2;
        mod.samples[i].loop_length = (uint32_t)rep_len   * 2;
    }

    mod.num_song_positions = read_flash(addr++);
    mod.restart_position   = read_flash(addr++);

    // --- Read order table; simultaneously find the highest pattern index ---
    uint8_t max_pat = 0;
    for (int i = 0; i < 128; i++) {
        mod.order[i] = read_flash(addr++);
        if (mod.order[i] > max_pat) max_pat = mod.order[i];
    }
    mod.num_patterns = max_pat + 1;

    addr += 4; // Skip 4-byte signature ("M.K.", "FLT4", "4CHN", etc.)
    mod.pattern_data_offset = addr;

    // --- Calculate absolute flash addresses for each sample's PCM data ---
    // Sample data immediately follows all pattern data.
    uint32_t s_offset = mod.pattern_data_offset + ((uint32_t)mod.num_patterns * 1024);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        mod.samples[i].data_offset = s_offset;
        s_offset += mod.samples[i].length;
    }

    // --- Initialise player state ---
    player.song_pos         = 0;
    player.pattern_row      = 0;
    player.tick             = 0;
    player.ticks_per_row    = 6;   // ProTracker default speed
    player.bpm              = 125; // ProTracker default tempo
    update_tick_timing();
    player.samples_remaining = 0;
}

// =========================================================================
// HARDWARE INITIALISATION
// =========================================================================

void hardware_init() {
    // --- I/O Direction ---
    DDRA  = 0xFF;   // PA0-PA7 ? Flash address bus A0-A7 (output)
    DDRB  = 0xFF;   // PB0-PB7 ? Flash address bus A8-A15 (output)
    DDRC  = 0x00;   // PC0-PC7 ? Flash data bus DQ0-DQ7 (input)
    PORTC = 0x00;   // Disable pull-ups on data bus inputs
    // PD0-PD2: A16-A18 (output), PD3: SER (output), PD4: OC1B/Right (output),
    // PD5: OC1A/Left (output), PD6: SRCLK (output), PD7: RCLK (output)
    DDRD  = 0xFF;

    // --- Audio PWM: Timer1 in 8-bit Fast PWM mode ---
    // OC1A (PD5) = Left channel, OC1B (PD4) = Right channel.
    // Non-inverted (clear on compare match, set at BOTTOM).
    // Carrier frequency = 16 MHz / 256 = 62.5 kHz (well above audible range).
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // 8-bit Fast PWM, non-inverted
    TCCR1B = (1 << WGM12)  | (1 << CS10);                   // Prescaler 1 (no division)
    OCR1A  = 128; // Idle: 50% duty cycle (silence = mid-rail)
    OCR1B  = 128;

    // --- Audio sample-rate interrupt: Timer0 CTC @ 16,000 Hz ---
    // Period = F_CPU / (Prescaler * SAMPLE_RATE) - 1 = 16000000 / (8 * 16000) - 1 = 124
    TCCR0  = (1 << WGM01) | (1 << CS01); // CTC mode, prescaler 8
    OCR0   = 124;                         // Compare value for 16 kHz interrupt rate
    TIMSK |= (1 << OCIE0);               // Enable Timer0 Output Compare Match interrupt
}

// =========================================================================
// MAIN ENTRY POINT
// =========================================================================
// Replaces Arduino's setup() + loop() pattern.
// hardware_init() = setup(), the while(1) body = loop().

int main(void) {
    cli();           // Disable interrupts during initialisation
    hardware_init(); // Configure I/O, Timer0 (ISR clock), Timer1 (PWM audio)
    mod_load();      // Parse MOD header from flash, prime player state
    sei();           // Enable interrupts ? Timer0 ISR now fires at 16 kHz
    audio_running = 1;

    // Main loop: continuously fill the audio ring-buffer ahead of the ISR.
    // All audio computation happens here so the ISR stays as short as possible.
    while (1) {
        if (!audio_running) continue;

        // Fill ring-buffer until it is full (one slot must remain empty to
        // distinguish "full" from "empty": wr+1 == rd means full).
        while (((wr_ptr + 1) & (BUF_SIZE - 1)) != rd_ptr) {

            // When the tick's worth of audio samples is exhausted, advance the sequencer.
            if (player.samples_remaining <= 0) {
                process_tick();          // Decode next pattern row / apply effects
                update_leds();           // Sync LED flash to note onsets
                player.samples_remaining = player.samples_per_tick;
            }
            player.samples_remaining--;

            // --- Mix four channels into stereo ---
            // ProTracker hard-pans: channels 1 & 4 (index 0 & 3) ? Left
            //                       channels 2 & 3 (index 1 & 2) ? Right
            int16_t left_mix  = 0;
            int16_t right_mix = 0;

            for (int ch = 0; ch < CHANNELS; ch++) {
                Channel* chn = &player.channel[ch];
                int8_t   s   = 0; // Signed 8-bit PCM sample from flash

                if (chn->period > 0 && chn->length > 0) {
                    uint32_t pos_int = chn->position >> 16; // Integer part of 16.16 position

                    // --- Loop / End-of-sample handling ---
                    if (pos_int >= chn->length) {
                        if (chn->loop_length > 2) {
                            // Looping sample: wrap position back into the loop region.
                            uint32_t over = pos_int - chn->loop_start;
                            pos_int = chn->loop_start + (over % chn->loop_length);
                            // Write corrected integer position back, preserving fractional bits.
                            chn->position = (pos_int << 16) | (chn->position & 0xFFFF);
                        } else {
                            // One-shot sample: stop the channel.
                            chn->period = 0;
                        }
                    }

                    if (chn->period > 0) {
                        // Fetch signed 8-bit PCM byte from flash and advance position.
                        s = (int8_t)read_flash(chn->data_offset + pos_int);
                        chn->position += chn->step;
                    }
                }

                // Apply volume scaling: volume 0-64, so shift right 6 bits (divide by 64).
                int16_t mixed = (s * chn->volume) >> 6;

                // Hard-pan as per ProTracker LRRL channel assignment.
                if (ch == 0 || ch == 3) left_mix  += mixed;
                else                    right_mix += mixed;
            }

            // Halve each mix to prevent clipping when two channels are at full volume,
            // then add 128 to convert from signed to unsigned (0-255) for PWM.
            left_mix  = (left_mix  >> 1) + 128;
            right_mix = (right_mix >> 1) + 128;

            audio_buf_L[wr_ptr] = (uint8_t)left_mix;
            audio_buf_R[wr_ptr] = (uint8_t)right_mix;

            wr_ptr = (wr_ptr + 1) & (BUF_SIZE - 1);
        }
    }

    return 0; // Never reached; satisfies the C standard for main()
}

// =========================================================================
// INTERRUPT SERVICE ROUTINE ? Timer0 Compare Match (fires @ 16,000 Hz)
// =========================================================================
// Writes the next buffered stereo sample frame to the PWM compare registers.
// Kept minimal to minimise jitter: just two register writes and a pointer bump.

ISR(TIMER0_COMP_vect) {
    if (rd_ptr != wr_ptr) {
        OCR1A = audio_buf_L[rd_ptr]; // Left  channel ? OC1A / PD5
        OCR1B = audio_buf_R[rd_ptr]; // Right channel ? OC1B / PD4
        rd_ptr = (rd_ptr + 1) & (BUF_SIZE - 1);
    }
}