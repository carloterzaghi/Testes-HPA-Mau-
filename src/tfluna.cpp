#include "tfluna.hpp"

#include "pico/stdlib.h"

TFLuna::TFLuna(uint8_t address, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin)
    : _address(address),
      _i2c_scl_pin(i2c_scl_pin),
      _i2c_sda_pin(i2c_sda_pin),
      data{} {
    i2c_init(i2c_default, TFLUNA_I2C_FREQUENCY);
    gpio_set_function(_i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(_i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_i2c_scl_pin);
    gpio_pull_up(_i2c_sda_pin);
}

bool TFLuna::read() {
    uint8_t register_address = TFLUNA_DATA_REGISTER;
    uint8_t buffer[TFLUNA_DATA_LENGTH];

    int write_result = i2c_write_blocking(
        i2c_default, _address, &register_address, 1, true);
    if (write_result != 1) {
        return false;
    }

    int read_result = i2c_read_blocking(
        i2c_default, _address, buffer, TFLUNA_DATA_LENGTH, false);
    if (read_result != TFLUNA_DATA_LENGTH) {
        return false;
    }
    data.distance_cm = static_cast<uint16_t>(buffer[0]) |
                       (static_cast<uint16_t>(buffer[1]) << 8);
    data.signal_strength = static_cast<uint16_t>(buffer[2]) |
                           (static_cast<uint16_t>(buffer[3]) << 8);
    uint16_t temperature_raw = static_cast<uint16_t>(buffer[4]) |
                               (static_cast<uint16_t>(buffer[5]) << 8);
    data.temperature_c = static_cast<int16_t>(temperature_raw / 100 - 256);

    return true;
}