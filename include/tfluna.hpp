#ifndef TFLUNA_HPP
#define TFLUNA_HPP

#include <stdint.h>

#include "i2c_bus.hpp"

#define TFLUNA_DEFAULT_ADDRESS 0x10
#define TFLUNA_I2C_FREQUENCY 400000
#define TFLUNA_DATA_REGISTER 0x00
#define TFLUNA_DATA_LENGTH 9

typedef struct {
    uint16_t distance_cm;
    uint16_t signal_strength;
    int16_t temperature_c;
} tfluna_data_t;

class TFLuna {
public:
    TFLuna(I2CBus& bus, uint8_t address);

    bool read();

    tfluna_data_t data;

private:
    I2CBus& _bus;
    uint8_t _address;
};

#endif