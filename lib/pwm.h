#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwm_init_(uint8_t pin);
void pwm_setup(uint8_t pin, float diviser, uint16_t wrap);
void pwm_start(uint8_t pin, uint16_t duty_cycle);

#endif