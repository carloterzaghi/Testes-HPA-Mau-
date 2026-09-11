#ifndef TFLUNA_HPP
#define TFLUNA_HPP

#include <stdint.h>

#include "hardware/i2c.h"

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
    TFLuna(uint8_t address, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin);

    bool read();

    tfluna_data_t data;

private:
    uint8_t _address;
    uint8_t _i2c_scl_pin;
    uint8_t _i2c_sda_pin;
};

#endif