#include "tfluna.hpp"

#include "pico/stdlib.h"

TFLuna::TFLuna(I2CBus& bus, uint8_t address)
    : _bus(bus),
    _address(address),
      data{} {
}

bool TFLuna::read() {
    uint8_t register_address = TFLUNA_DATA_REGISTER;
    uint8_t buffer[TFLUNA_DATA_LENGTH];

    int write_result = _bus.write(_address, &register_address, 1, true);
    if (write_result != 1) {
        return false;
    }

    int read_result = _bus.read(_address, buffer, TFLUNA_DATA_LENGTH, false);
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