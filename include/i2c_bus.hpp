#ifndef I2C_BUS_HPP
#define I2C_BUS_HPP

#include <stddef.h>
#include <stdint.h>

#include "hardware/i2c.h"

class I2CBus {
public:
    I2CBus(uint8_t scl_pin,
           uint8_t sda_pin,
           uint32_t frequency = 400000,
           i2c_inst_t* instance = i2c_default);

    int write(uint8_t address,
              const uint8_t* data,
              size_t length,
              bool no_stop = false);
    int read(uint8_t address,
             uint8_t* data,
             size_t length,
             bool no_stop = false);

    i2c_inst_t* instance() const;

private:
    i2c_inst_t* _instance;
};

#endif