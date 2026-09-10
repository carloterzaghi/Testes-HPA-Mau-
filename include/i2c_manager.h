#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>

typedef struct {
    i2c_inst_t *i2c;
    uint8_t scl_pin;
    uint8_t sda_pin;

} i2c_manager;


#endif