#include "as5600.hpp"

#include "pico/time.h"

namespace {
constexpr uint16_t AS5600_MAX_ANGLE = 4096;
constexpr uint16_t AS5600_HALF_TURN = AS5600_MAX_ANGLE / 2;
constexpr int32_t AS5600_MDEG_PER_TURN = 360000;
constexpr uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0C;
}

AS5600::AS5600(I2CBus& bus, uint8_t address)
        : _bus(bus),
            _address(address),
            _last_sample_time_us(0),
      _latest_angle(0),
      _latest_speed_rpm_milli(0),
      _sample_ready(false),
      _initialized(false),
      data{} {
}

bool AS5600::init() {
    _initialized = true;
    return read();
}

bool AS5600::read() {
    if (!_initialized) {
        return false;
    }

    uint8_t register_address = AS5600_RAW_ANGLE_REGISTER;
    uint8_t angle_bytes[2];
    if (_bus.write(_address, &register_address, 1, true) != 1 ||
        _bus.read(_address, angle_bytes, sizeof(angle_bytes)) !=
            static_cast<int>(sizeof(angle_bytes))) {
        return false;
    }

    uint16_t angle = static_cast<uint16_t>(
        ((static_cast<uint16_t>(angle_bytes[0]) << 8) | angle_bytes[1]) &
        (AS5600_MAX_ANGLE - 1));

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    if (!_sample_ready) {
        _latest_speed_rpm_milli = 0;
    } else {
        uint32_t elapsed_us = static_cast<uint32_t>(now_us - _last_sample_time_us);
        int32_t delta = static_cast<int32_t>(angle) - _latest_angle;
        if (delta > AS5600_HALF_TURN) {
            delta -= AS5600_MAX_ANGLE;
        } else if (delta < -AS5600_HALF_TURN) {
            delta += AS5600_MAX_ANGLE;
        }
        if (elapsed_us > 0) {
            _latest_speed_rpm_milli = static_cast<int32_t>(
                (static_cast<int64_t>(delta) * 60000000LL * 1000LL) /
                (static_cast<int64_t>(AS5600_MAX_ANGLE) * elapsed_us));
        }
    }

    _latest_angle = angle;
    _last_sample_time_us = now_us;
    _sample_ready = true;
    data.raw_angle = angle;
    data.angle_mdeg = static_cast<uint16_t>(
        (static_cast<uint32_t>(angle) * AS5600_MDEG_PER_TURN) /
        AS5600_MAX_ANGLE);
    data.speed_rpm_milli = _latest_speed_rpm_milli;
    return true;
}