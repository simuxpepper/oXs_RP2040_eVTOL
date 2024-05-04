/*#include "../include/common.h"

// 1. Set configuration
hx711_config_t hxcfg;
hx711_get_default_config(&hxcfg);

hxcfg.clock_pin = 14; //GPIO pin connected to HX711 clock pin
hxcfg.data_pin = 15; //GPIO pin connected to HX711 data pin

//by default, the underlying PIO program will run on pio0
//if you need to change this, you can do:
//hxcfg.pio = pio1;

// 2. Initialise
hx711_init(&hx, &hxcfg);

// 3. Power up the hx711 and set gain on chip
hx711_power_up(&hx, hx711_gain_128);

// 4. This step is optional. Only do this if you want to
// change the gain AND save it to the HX711 chip
//
// hx711_set_gain(&hx, hx711_gain_64);
// hx711_power_down(&hx);
// hx711_wait_power_down();
// hx711_power_up(&hx, hx711_gain_64);

// 5. Wait for readings to settle
hx711_wait_settle(rate);

// 6. Read values
// You can now...

// wait (block) until a value is obtained
printf("blocking value: %li\n", hx711_get_value(&hx));

// or use a timeout
int32_t val;
const uint timeout = 250000; //microseconds
if(hx711_get_value_timeout(&hx, timeout, &val)) {
    // value was obtained within the timeout period
    printf("timeout value: %li\n", val);
}
else {
    printf("value was not obtained within the timeout period\n");
}

// or see if there's a value, but don't block if there isn't one ready
if(hx711_get_value_noblock(&hx, &val)) {
    printf("noblock value: %li\n", val);
}
else {
    printf("value was not present\n");
}



//6. Stop communication with HX711
hx711_close(&hx);*/