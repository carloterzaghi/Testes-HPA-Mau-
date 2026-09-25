#include "i2c_bus.hpp"

#include "pico/stdlib.h"

I2CBus::I2CBus(uint8_t scl_pin,
               uint8_t sda_pin,
               uint32_t frequency,
               i2c_inst_t* instance)
    : _instance(instance) {
    i2c_init(_instance, frequency);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(scl_pin);
    gpio_pull_up(sda_pin);
}

int I2CBus::write(uint8_t address,
                 const uint8_t* data,
                 size_t length,
                 bool no_stop) {
    return i2c_write_blocking(_instance, address, data, length, no_stop);
}

int I2CBus::read(uint8_t address,
                uint8_t* data,
                size_t length,
                bool no_stop) {
    return i2c_read_blocking(_instance, address, data, length, no_stop);
}

i2c_inst_t* I2CBus::instance() const {
    return _instance;
}