// Note playing demo for the Pico
// This example uses the SPI interface to output a 16-bit signal to a DAC.
// The DAC is a MCP4822, which has two channels.
// The example uses a timer interrupt to generate a sine wave at a given frequency.
// The frequency is set by the phase increment, which is calculated based on the desired frequency and the sample rate.
// The example also includes a simple gamepad interface to change the frequency of the output signal.
// The gamepad is connected via USB and uses the TinyUSB library.
// Dual Shock 4 or dual sensor gamepad is supported.
// The gamepad buttons are mapped to different frequencies.
// The gamepad buttons are:
//  - DOWN: DO
//  - LEFT: RE
//  - UP: MI
//  - RIGHT: FA
//  - SELECT: SOL
//  - START: LA
//  - B: SI
//  - A: DO
// The example includes a simple song that plays a series of notes using the sine wave generator.
 
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

#include <tusb.h>
#include <gamepad.h>
#ifndef DWORD
typedef unsigned long DWORD;
#endif /* !DWORD */
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)


// DDS parameters
#define two32 4294967296.0 // 2^32
#define Fs 44100 // 50000
#define DELAY 23 // 20 // 1/Fs (in microseconds): dus 20 microseconds * 50000 = 1 seconde
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (0.0 * two32) / Fs; // was 800.0

// SPI data
uint16_t DAC_data; // output value

// DAC parameters
//  A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0
#define LDAC_GPIO 3
// GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
// Add instrument types
enum Instrument {
    INSTR_SINE,
    INSTR_SQUARE,
    INSTR_TRIANGLE
};

// Update Note struct to include instrument
struct Note {
    double freq;
    int duration;
    Instrument instrument;
};

// Three lookup tables: sine, square, triangle
volatile int sin_table[sine_table_size];
volatile int square_table[sine_table_size];
volatile int triangle_table[sine_table_size];
// Define note frequencies (Hz)
#define NOTE_DO  263.0
#define NOTE_RE  294.0
#define NOTE_MI  330.0
#define NOTE_FA  349.0
#define NOTE_SOL 392.0
#define NOTE_LA  440.0
#define NOTE_SI  494.0
#define NOTE_DO2 523.0
#define NOTE_SILENT 0.0

// Example song: cycle through three instruments
Note song[] = {
    {NOTE_MI, 400, INSTR_SINE},     // Sine
    {NOTE_RE, 400, INSTR_SQUARE},   // Square
    {NOTE_DO, 400, INSTR_TRIANGLE}, // Triangle
    {NOTE_RE, 400, INSTR_SINE},
    {NOTE_MI, 400, INSTR_SQUARE},
    {NOTE_MI, 400, INSTR_TRIANGLE},
    {NOTE_MI, 800, INSTR_SINE},
    {NOTE_RE, 400, INSTR_SQUARE},
    {NOTE_RE, 400, INSTR_TRIANGLE},
    {NOTE_RE, 800, INSTR_SINE},
    {NOTE_MI, 400, INSTR_SQUARE},
    {NOTE_SOL, 400, INSTR_TRIANGLE},
    {NOTE_SOL, 800, INSTR_SINE},
    {NOTE_MI, 400, INSTR_SQUARE},
    {NOTE_RE, 400, INSTR_TRIANGLE},
    {NOTE_DO, 400, INSTR_SINE},
    {NOTE_RE, 400, INSTR_SQUARE},
    {NOTE_MI, 400, INSTR_TRIANGLE},
    {NOTE_MI, 400, INSTR_SINE},
    {NOTE_MI, 400, INSTR_SQUARE},
    {NOTE_RE, 400, INSTR_TRIANGLE},
    {NOTE_RE, 400, INSTR_SINE},
    {NOTE_MI, 400, INSTR_SQUARE},
    {NOTE_RE, 400, INSTR_TRIANGLE},
    {NOTE_DO, 1200, INSTR_SINE}
};
const int song_length = sizeof(song) / sizeof(song[0]);
// Alarm ISR
static void alarm_irq(void)
{

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // DDS phase and sine table lookup
    phase_accum_main += phase_incr_main;
    DAC_data = (DAC_config_chan_A | ((sin_table[phase_accum_main >> 24] + 2048) & 0xffff));

    // Perform an SPI transaction
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    DAC_data = (DAC_config_chan_B | ((sin_table[phase_accum_main >> 24] + 2048) & 0xffff));

    // Perform an SPI transaction
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);


    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

volatile const uint8_t* audio_stream = NULL;
volatile size_t audio_stream_len = 0;
volatile size_t audio_stream_idx = 0;
volatile bool audio_stream_playing = false;

// --- Call this to start playback ---
void start_audio_stream(const uint8_t* data, size_t length) {
    audio_stream = data;
    audio_stream_len = length;
    audio_stream_idx = 0;
    audio_stream_playing = true;
}

// --- In your alarm_irq ---
static void alarm_irq_stream(void)
{
    gpio_put(ISR_GPIO, 1);
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (audio_stream_playing && audio_stream && audio_stream_idx < audio_stream_len) {
        // 8-bit unsigned PCM, map 0–255 to 0–4095
        int dac_value = (audio_stream[audio_stream_idx] * 4095) / 255;
        DAC_data = (DAC_config_chan_A | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        DAC_data = (DAC_config_chan_B | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        audio_stream_idx++;
        if (audio_stream_idx >= audio_stream_len) {
            audio_stream_playing = false;
            // Optionally, silence output
            DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
            spi_write16_blocking(SPI_PORT, &DAC_data, 1);
            DAC_data = (DAC_config_chan_B | (2048 & 0xffff));
            spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        }
    }
    gpio_put(ISR_GPIO, 0);
}
void getgamepadstate()
{
    static constexpr int LEFT = 1 << 6;
    static constexpr int RIGHT = 1 << 7;
    static constexpr int UP = 1 << 4;
    static constexpr int DOWN = 1 << 5;
    static constexpr int SELECT = 1 << 2;
    static constexpr int START = 1 << 3;
    static constexpr int A = 1 << 0;
    static constexpr int B = 1 << 1;

    static DWORD prevButtons{};
    auto &gp = io::getCurrentGamePadState(0);
    int v = (gp.buttons & io::GamePadState::Button::LEFT ? LEFT : 0) |
            (gp.buttons & io::GamePadState::Button::RIGHT ? RIGHT : 0) |
            (gp.buttons & io::GamePadState::Button::UP ? UP : 0) |
            (gp.buttons & io::GamePadState::Button::DOWN ? DOWN : 0) |
            (gp.buttons & io::GamePadState::Button::A ? A : 0) |
            (gp.buttons & io::GamePadState::Button::B ? B : 0) |
            (gp.buttons & io::GamePadState::Button::SELECT ? SELECT : 0) |
            (gp.buttons & io::GamePadState::Button::START ? START : 0) |
            0;
    auto pushed = v & ~prevButtons;
    auto released = prevButtons & ~v;
    if (pushed)
    {
        if (pushed & DOWN)
        {
            phase_incr_main = (263.0 * two32) / Fs; // DO
        }
        else if (pushed & LEFT)
        {
            phase_incr_main = (294.0 * two32) / Fs; // RE
        }
        else if (pushed & UP)
        {
            phase_incr_main = (330.0 * two32) / Fs; // MI
        }
        else if (pushed & RIGHT)
        {
            phase_incr_main = (349.0 * two32) / Fs; // FA
        }
        else if (pushed & SELECT)
        {
            phase_incr_main = (392.0 * two32) / Fs; // SOL
        }
        else if (pushed & START)
        {
            phase_incr_main = (440.0 * two32) / Fs; // LA
        }
        if (pushed & B)
        {
            phase_incr_main = (494.0 * two32) / Fs; // SI
        }
        else if (pushed & A)
        {
            phase_incr_main = (523.0 * two32) / Fs; // DO
        }
    }
    if (released)
    {
        phase_incr_main = 0;
    }
    prevButtons = v;
    // if (pushed)
    // {
    //     printf("phase_incr_main = %f\n", phase_incr_main * Fs / two32);
    // }
}


// Song: Mary Had a Little Lamb (notes and durations in ms)

#if false
void play_song() {
    for (int i = 0; i < song_length; ++i) {
        if (song[i].freq > 0.0) {
            phase_incr_main = (song[i].freq * two32) / Fs;
        } else {
            phase_incr_main = 0;
        }
        busy_wait_ms(song[i].duration);
        phase_incr_main = 0; // Silence between notes
        busy_wait_ms(50);
    }
}
#endif

void play_song() {
    for (int i = 0; i < song_length; ++i) {
        if (song[i].freq > 0.0) {
            unsigned int phase_incr = (song[i].freq * two32) / Fs;
            unsigned int phase_accum = 0;
            int duration_samples = (Fs * song[i].duration) / 1000;
            for (int j = 0; j < duration_samples; ++j) {
                phase_accum += phase_incr;
                int sample;
                switch (song[i].instrument) {
                    case INSTR_SINE:
                        sample = sin_table[phase_accum >> 24];
                        break;
                    case INSTR_SQUARE:
                        sample = square_table[phase_accum >> 24];
                        break;
                    case INSTR_TRIANGLE:
                        sample = triangle_table[phase_accum >> 24];
                        break;
                    default:
                        sample = 0;
                }
                DAC_data = (DAC_config_chan_A | ((sample + 2048) & 0xffff));
                spi_write16_blocking(SPI_PORT, &DAC_data, 1);
                DAC_data = (DAC_config_chan_B | ((sample + 2048) & 0xffff));
                spi_write16_blocking(SPI_PORT, &DAC_data, 1);
                busy_wait_us(20); // 1/Fs = 20us
            }
        }
        // Silence between notes
        DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
         DAC_data = (DAC_config_chan_B | (2048 & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        busy_wait_ms(50);
    }
}

void play_multitone(double freq1, Instrument inst1, double freq2, Instrument inst2, int duration_ms) {
    unsigned int phase_accum1 = 0, phase_accum2 = 0;
    unsigned int phase_incr1 = (freq1 * two32) / Fs;
    unsigned int phase_incr2 = (freq2 * two32) / Fs;
    int samples = (Fs * duration_ms) / 1000;
    for (int i = 0; i < samples; ++i) {
        phase_accum1 += phase_incr1;
        phase_accum2 += phase_incr2;
        int sample1, sample2;
        switch (inst1) {
            case INSTR_SINE: sample1 = sin_table[phase_accum1 >> 24]; break;
            case INSTR_SQUARE: sample1 = square_table[phase_accum1 >> 24]; break;
            case INSTR_TRIANGLE: sample1 = triangle_table[phase_accum1 >> 24]; break;
            default: sample1 = 0;
        }
        switch (inst2) {
            case INSTR_SINE: sample2 = sin_table[phase_accum2 >> 24]; break;
            case INSTR_SQUARE: sample2 = square_table[phase_accum2 >> 24]; break;
            case INSTR_TRIANGLE: sample2 = triangle_table[phase_accum2 >> 24]; break;
            default: sample2 = 0;
        }
        // Average the two samples to avoid overflow
        int sample = (sample1 + sample2) / 2;
        DAC_data = (DAC_config_chan_A | ((sample + 2048) & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        busy_wait_us(20); // 1/Fs = 20us
    }
    // Silence after chord
    DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
}

void play_multitone_stereo(double freqL, Instrument instL, double freqR, Instrument instR, int duration_ms) {
    unsigned int phase_accumL = 0, phase_accumR = 0;
    unsigned int phase_incrL = (freqL * two32) / Fs;
    unsigned int phase_incrR = (freqR * two32) / Fs;
    int samples = (Fs * duration_ms) / 1000;
    for (int i = 0; i < samples; ++i) {
        phase_accumL += phase_incrL;
        phase_accumR += phase_incrR;
        int sampleL, sampleR;
        switch (instL) {
            case INSTR_SINE: sampleL = sin_table[phase_accumL >> 24]; break;
            case INSTR_SQUARE: sampleL = square_table[phase_accumL >> 24]; break;
            case INSTR_TRIANGLE: sampleL = triangle_table[phase_accumL >> 24]; break;
            default: sampleL = 0;
        }
        switch (instR) {
            case INSTR_SINE: sampleR = sin_table[phase_accumR >> 24]; break;
            case INSTR_SQUARE: sampleR = square_table[phase_accumR >> 24]; break;
            case INSTR_TRIANGLE: sampleR = triangle_table[phase_accumR >> 24]; break;
            default: sampleR = 0;
        }
        // Output to both channels
        DAC_data = (DAC_config_chan_A | ((sampleL + 2048) & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        DAC_data = (DAC_config_chan_B | ((sampleR + 2048) & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        busy_wait_us(20); // 1/Fs = 20us
    }
    // Silence after chord
    DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
    DAC_data = (DAC_config_chan_B | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
}

void play_multitone_song() {
    // Each entry: freq1, inst1, freq2, inst2, duration_ms
    struct Chord {
        double freq1;
        Instrument inst1;
        double freq2;
        Instrument inst2;
        int duration;
    };
    Chord chords[] = {
        {NOTE_DO, INSTR_SINE, NOTE_MI, INSTR_SQUARE, 500},
        {NOTE_FA, INSTR_TRIANGLE, NOTE_LA, INSTR_SINE, 500},
        {NOTE_SOL, INSTR_SQUARE, NOTE_DO2, INSTR_TRIANGLE, 500},
        {NOTE_MI, INSTR_SINE, NOTE_SI, INSTR_SQUARE, 700},
        {NOTE_DO, INSTR_TRIANGLE, NOTE_SOL, INSTR_SINE, 1000}
    };
    int num_chords = sizeof(chords) / sizeof(chords[0]);
    for (int i = 0; i < num_chords; ++i) {
        play_multitone(
            chords[i].freq1, chords[i].inst1,
            chords[i].freq2, chords[i].inst2,
            chords[i].duration
        );
        busy_wait_ms(100); // Short pause between chords
    }
}
void play_multitone_stereo_song() {
    // Each entry: left freq, left inst, right freq, right inst, duration_ms
    struct StereoNote {
        double freqL;
        Instrument instL;
        double freqR;
        Instrument instR;
        int duration;
    };
    StereoNote stereo_song[] = {
        {NOTE_DO, INSTR_SINE, NOTE_MI, INSTR_SQUARE, 400},
        {NOTE_RE, INSTR_TRIANGLE, NOTE_FA, INSTR_SINE, 400},
        {NOTE_MI, INSTR_SQUARE, NOTE_SOL, INSTR_TRIANGLE, 400},
        {NOTE_FA, INSTR_SINE, NOTE_LA, INSTR_SQUARE, 400},
        {NOTE_SOL, INSTR_TRIANGLE, NOTE_SI, INSTR_SINE, 400},
        {NOTE_LA, INSTR_SQUARE, NOTE_DO2, INSTR_TRIANGLE, 800}
    };
    int num_notes = sizeof(stereo_song) / sizeof(stereo_song[0]);
    for (int i = 0; i < num_notes; ++i) {
        play_multitone_stereo(
            stereo_song[i].freqL, stereo_song[i].instL,
            stereo_song[i].freqR, stereo_song[i].instR,
            stereo_song[i].duration
        );
        busy_wait_ms(100); // Short pause between notes
    }
}
void play_stereo_song_alt() {
    // Define a melody to alternate
    struct SimpleNote {
        double freq;
        Instrument inst;
        int duration;
    };
    SimpleNote melody[] = {
        {NOTE_DO, INSTR_SINE, 300},
        {NOTE_RE, INSTR_SQUARE, 300},
        {NOTE_MI, INSTR_TRIANGLE, 300},
        {NOTE_FA, INSTR_SINE, 300},
        {NOTE_SOL, INSTR_SQUARE, 300},
        {NOTE_LA, INSTR_TRIANGLE, 300},
        {NOTE_SI, INSTR_SINE, 300},
        {NOTE_DO2, INSTR_SQUARE, 600}
    };
    int num_notes = sizeof(melody) / sizeof(melody[0]);
    for (int i = 0; i < num_notes; ++i) {
        if (i % 2 == 0) {
            // Even index: play on left, silence on right
            play_multitone_stereo(
                melody[i].freq, melody[i].inst,
                NOTE_SILENT, INSTR_SINE,
                melody[i].duration
            );
        } else {
            // Odd index: play on right, silence on left
            play_multitone_stereo(
                NOTE_SILENT, INSTR_SINE,
                melody[i].freq, melody[i].inst,
                melody[i].duration
            );
        }
        busy_wait_ms(80); // Short pause between notes
    }
}
void play_byte_stream(const uint8_t* data, size_t length, unsigned sample_rate_hz) {
    unsigned delay_us = 1000000 / sample_rate_hz; // microseconds per sample
    delay_us = 18;
    printf("Playing %zu bytes at %u Hz (%u)\n", length, sample_rate_hz, delay_us);
    const int gain = 10; // Increase for more volume, but avoid clipping
    for (size_t i = 0; i < length; ++i) {
        // Center at 128, scale, and re-center at 2048
        int centered = ((int)data[i] - 128) * gain;
        int dac_value = 2048 + centered;
        if (dac_value < 0) dac_value = 0;
        if (dac_value > 4095) dac_value = 4095;
        DAC_data = (DAC_config_chan_A | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        DAC_data = (DAC_config_chan_B | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        busy_wait_us(delay_us); 
    }
    DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
     DAC_data = (DAC_config_chan_B | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
}

void play_byte_streamMax(const uint8_t* data, size_t length, unsigned sample_rate_hz) {
    unsigned delay_us = 1000000 / sample_rate_hz; // microseconds per sample
    for (size_t i = 0; i < length; ++i) {
        // Map 0–255 directly to 0–4095
        int dac_value = (data[i] * 4095) / 255;
        DAC_data = (DAC_config_chan_A | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
         DAC_data = (DAC_config_chan_B | (dac_value & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);
        busy_wait_us(delay_us);
    }
    DAC_data = (DAC_config_chan_A | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
    DAC_data = (DAC_config_chan_B | (2048 & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
}
extern char sound[];
extern signed int sound_len;
int main()
{
    stdio_init_all();
  
    

    busy_wait_ms(1000);
    printf("Play Tones Demo\n");
    printf("Init USB\n");
    tusb_init();
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)

    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);


    gpio_init(LDAC_GPIO);
    gpio_set_dir(LDAC_GPIO, GPIO_OUT);
    gpio_put(LDAC_GPIO, 0);
    
    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // === build the sine lookup table =======
    // scaled to produce values between 0 and 4096
    int ii;
   for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
        square_table[ii] = (ii < sine_table_size / 2) ? 2047 : -2047;
        // Triangle wave: ramps up and down linearly
        if (ii < sine_table_size / 2)
            triangle_table[ii] = (int)(-2047 + (ii * (4094.0 / (sine_table_size / 2))));
        else
            triangle_table[ii] = (int)(2047 - ((ii - sine_table_size / 2) * (4094.0 / (sine_table_size / 2))));
    }
    // wait for keypress
    // printf("Press any key to start\n");
    // char c = getchar();
    printf("Starting...\n");
    
    // play_byte_stream((const uint8_t*)sound, sound_len, 44100);
    // printf("Playing song, two channels\n");
    // play_song() ;
    // printf("Playing multitone song\n");
    // play_multitone_song();
    // printf("Playing multitone stereo song\n");
    // play_multitone_stereo_song();
    // printf("Playing alternate over left and right\n");
    // play_stereo_song_alt();
    
     

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq_stream);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
    while(true) {
        if ( !audio_stream_playing ) {
            printf("Forever Playing Mario!\n");
            start_audio_stream((const uint8_t*)sound, sound_len);
        }
        busy_wait_us(10);
    }
    printf("Press buttons to play notes\n");
    while (1)
    {
        tuh_task();
        getgamepadstate();
    }
    return 0;
}
