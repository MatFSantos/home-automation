#include "push_button.h"

void init_push_button(uint8_t pin){
    // initialize push button pin as input
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    // enable pull-up resistor
    gpio_pull_up(pin);
}

bool push_button_get(uint8_t pin, bool debounce){
    if (gpio_get(pin) == 0 && debounce)
        sleep_ms(DEBOUNCE_TIME);
    return gpio_get(pin);
}