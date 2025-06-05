#ifndef PUSH_BUTTON_H
#define PUSH_BUTTON_H

#include <stdint.h>
#include "pico/stdlib.h"

#define PIN_BUTTON_A 5
#define PIN_BUTTON_B 6
#define PIN_BUTTON_J 22
#define DEBOUNCE_TIME 100

/**
 * @file push_button.h
 * @brief Library file that contains the configuration and definitions of Push buttons A and B.
 * 
 * @author Mateus F Santos
 * @date 01.02.2025
 */

void init_push_button(uint8_t pin);

bool push_button_get(uint8_t pin, bool debounce);

#endif