#include "pwm.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

void pwm_init_(uint8_t pin){
    gpio_set_function(pin, GPIO_FUNC_PWM);
}
void pwm_setup(uint8_t pin, float diviser, uint16_t wrap){
    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_set_clkdiv(slice,diviser);

    pwm_set_wrap(slice, wrap);
}

void pwm_start(uint8_t pin, uint16_t duty_cycle){
    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_set_gpio_level(pin, duty_cycle);

    pwm_set_enabled(slice, true);
}