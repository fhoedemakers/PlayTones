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
#define ALARM_IRQ TIMER_IRQ_0

// DDS parameters
#define two32 4294967296.0 // 2^32
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds): dus 20 microseconds * 50000 = 1 seconde
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0 * two32) / Fs; //

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

// GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

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

    // De-assert the GPIO when we leave the interrupt
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
}
int main()
{
    stdio_init_all();
    printf("Play Tones Demo\n");
    tusb_init();
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)

    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

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
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Nothing happening here
    while (1)
    {
        tuh_task();
        getgamepadstate();
    }
    return 0;
}
