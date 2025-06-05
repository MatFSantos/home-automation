#include "led.h"

void init_led(uint8_t pin){
    // inicialyze LED and turn off
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
}

void led_manipulate(uint8_t pin, bool on) {
    gpio_put(pin, on);  // on/off the LED based on the input value (true = on, false = off)
}

void rgb_led_manipulate(bool red, bool green, bool blue){
    led_manipulate(PIN_RED_LED, red);
    led_manipulate(PIN_GREEN_LED, green);
    led_manipulate(PIN_BLUE_LED, blue);
}
