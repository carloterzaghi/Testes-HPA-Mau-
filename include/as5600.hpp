#ifndef AS5600_HPP
#define AS5600_HPP

#include <stdint.h>

#include "i2c_bus.hpp"
#include "pico/types.h"

typedef struct {
    uint16_t raw_angle;
    uint16_t angle_mdeg;
    int32_t speed_rpm_milli;
} as5600_data_t;

class AS5600 {
public:
    explicit AS5600(I2CBus& bus, uint8_t address = 0x36);

    bool init();
    bool read();
    as5600_data_t data;

private:
    I2CBus& _bus;
    uint8_t _address;
    uint64_t _last_sample_time_us;
    uint16_t _latest_angle;
    int32_t _latest_speed_rpm_milli;
    bool _sample_ready;
    bool _initialized;
};

#endif