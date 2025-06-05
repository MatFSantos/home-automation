#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>
#include "pico/stdlib.h"

#define PIN_RED_LED 13
#define PIN_BLUE_LED 12
#define PIN_GREEN_LED 11

#define init_rgb_led() do {\
    init_led(PIN_BLUE_LED); \
    init_led(PIN_GREEN_LED); \
    init_led(PIN_RED_LED); \
} while (0)


/**
 * @file rgb_led.h
 * @brief Library file that contains the configuration and definitions of RGB LED
 * 
 * @author Mateus F Santos
 * @date 01.02.2025
*/

/**
 * 
 */
void init_led(uint8_t pin);


/**
 * 
 */
void rgb_led_manipulate(bool red, bool green, bool blue);

void led_manipulate(uint8_t pin, bool on);

void rgb_led_manipulate(bool red, bool green, bool blue);

#endif