#include "bmp280.hpp"

#include "pico/stdlib.h"

namespace {
constexpr uint8_t BMP280_REGISTER_CALIBRATION = 0x88;
constexpr uint8_t BMP280_REGISTER_CHIP_ID = 0xD0;
constexpr uint8_t BMP280_REGISTER_RESET = 0xE0;
constexpr uint8_t BMP280_REGISTER_CONFIG = 0xF5;
constexpr uint8_t BMP280_REGISTER_CTRL_MEAS = 0xF4;
constexpr uint8_t BMP280_REGISTER_DATA = 0xF7;
constexpr uint8_t BMP280_RESET_VALUE = 0xB6;

uint16_t read_u16_le(const uint8_t* buffer) {
    return static_cast<uint16_t>(buffer[0]) |
           (static_cast<uint16_t>(buffer[1]) << 8);
}

int16_t read_i16_le(const uint8_t* buffer) {
    return static_cast<int16_t>(read_u16_le(buffer));
}
}

BMP280::BMP280(I2CBus& bus, uint8_t address)
    : _bus(bus),
      _address(address),
      _initialized(false),
      _dig_t1(0),
      _dig_t2(0),
      _dig_t3(0),
      _dig_p1(0),
      _dig_p2(0),
      _dig_p3(0),
      _dig_p4(0),
      _dig_p5(0),
      _dig_p6(0),
      _dig_p7(0),
      _dig_p8(0),
      _dig_p9(0),
      _t_fine(0),
      data{} {
}

bool BMP280::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if (_bus.write(_address, &reg, 1, true) != 1) {
        return false;
    }
    return _bus.read(_address, buffer, length, false) == length;
}

bool BMP280::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[] = {reg, value};
    return _bus.write(_address, buffer, sizeof(buffer), false) == sizeof(buffer);
}

bool BMP280::load_calibration() {
    uint8_t calibration[24];
    if (!read_registers(BMP280_REGISTER_CALIBRATION,
                        calibration,
                        sizeof(calibration))) {
        return false;
    }

    _dig_t1 = read_u16_le(&calibration[0]);
    _dig_t2 = read_i16_le(&calibration[2]);
    _dig_t3 = read_i16_le(&calibration[4]);
    _dig_p1 = read_u16_le(&calibration[6]);
    _dig_p2 = read_i16_le(&calibration[8]);
    _dig_p3 = read_i16_le(&calibration[10]);
    _dig_p4 = read_i16_le(&calibration[12]);
    _dig_p5 = read_i16_le(&calibration[14]);
    _dig_p6 = read_i16_le(&calibration[16]);
    _dig_p7 = read_i16_le(&calibration[18]);
    _dig_p8 = read_i16_le(&calibration[20]);
    _dig_p9 = read_i16_le(&calibration[22]);
    return _dig_p1 != 0;
}

bool BMP280::init() {
    uint8_t chip_id;
    if (!read_registers(BMP280_REGISTER_CHIP_ID, &chip_id, 1) ||
        chip_id != BMP280_CHIP_ID) {
        return false;
    }

    if (!write_register(BMP280_REGISTER_RESET, BMP280_RESET_VALUE)) {
        return false;
    }
    sleep_ms(2);

    if (!load_calibration()) {
        return false;
    }

    // Temperature and pressure x1 oversampling, normal operating mode.
    if (!write_register(BMP280_REGISTER_CTRL_MEAS, 0x27)) {
        return false;
    }
    // 1 second standby time, filter disabled, 4-wire SPI mode.
    if (!write_register(BMP280_REGISTER_CONFIG, 0xA0)) {
        return false;
    }

    _initialized = true;
    return true;
}

bool BMP280::read() {
    if (!_initialized) {
        return false;
    }

    uint8_t buffer[6];
    if (!read_registers(BMP280_REGISTER_DATA, buffer, sizeof(buffer))) {
        return false;
    }

    int32_t adc_pressure = (static_cast<int32_t>(buffer[0]) << 12) |
                           (static_cast<int32_t>(buffer[1]) << 4) |
                           (buffer[2] >> 4);
    int32_t adc_temperature = (static_cast<int32_t>(buffer[3]) << 12) |
                              (static_cast<int32_t>(buffer[4]) << 4) |
                              (buffer[5] >> 4);

    int32_t var1 = ((((adc_temperature >> 3) -
                      (static_cast<int32_t>(_dig_t1) << 1)) * _dig_t2) >> 11);
    int32_t var2 = (((((adc_temperature >> 4) - _dig_t1) *
                      ((adc_temperature >> 4) - _dig_t1)) >> 12) *
                    _dig_t3) >> 14;
    _t_fine = var1 + var2;
    data.temperature_centi_c = (_t_fine * 5 + 128) >> 8;

    int64_t pressure_var1 = static_cast<int64_t>(_t_fine) - 128000;
    int64_t pressure_var2 = pressure_var1 * pressure_var1 * _dig_p6;
    pressure_var2 += (pressure_var1 * _dig_p5) << 17;
    pressure_var2 += static_cast<int64_t>(_dig_p4) << 35;
    pressure_var1 = ((pressure_var1 * pressure_var1 * _dig_p3) >> 8) +
                    ((pressure_var1 * _dig_p2) << 12);
    pressure_var1 = (((static_cast<int64_t>(1) << 47) + pressure_var1) *
                     _dig_p1) >> 33;
    if (pressure_var1 == 0) {
        return false;
    }

    int64_t pressure = 1048576 - adc_pressure;
    pressure = (((pressure << 31) - pressure_var2) * 3125) / pressure_var1;
    pressure_var1 = (static_cast<int64_t>(_dig_p9) *
                     (pressure >> 13) * (pressure >> 13)) >> 25;
    pressure_var2 = (static_cast<int64_t>(_dig_p8) * pressure) >> 19;
    pressure = ((pressure + pressure_var1 + pressure_var2) >> 8) +
               (static_cast<int64_t>(_dig_p7) << 4);
    data.pressure_pa = static_cast<uint32_t>(pressure >> 8);
    return true;
}

bool BMP280::initialized() const {
    return _initialized;
}