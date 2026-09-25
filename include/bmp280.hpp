#ifndef BMP280_HPP
#define BMP280_HPP

#include <stdint.h>

#include "i2c_bus.hpp"

#define BMP280_ADDRESS_LOW 0x76
#define BMP280_ADDRESS_HIGH 0x77
#define BMP280_CHIP_ID 0x58

typedef struct {
    int32_t temperature_centi_c;
    uint32_t pressure_pa;
} bmp280_data_t;

class BMP280 {
public:
    BMP280(I2CBus& bus, uint8_t address = BMP280_ADDRESS_LOW);

    bool init();
    bool read();
    bool initialized() const;

    bmp280_data_t data;

private:
    I2CBus& _bus;
    uint8_t _address;
    bool _initialized;

    uint16_t _dig_t1;
    int16_t _dig_t2;
    int16_t _dig_t3;
    uint16_t _dig_p1;
    int16_t _dig_p2;
    int16_t _dig_p3;
    int16_t _dig_p4;
    int16_t _dig_p5;
    int16_t _dig_p6;
    int16_t _dig_p7;
    int16_t _dig_p8;
    int16_t _dig_p9;
    int32_t _t_fine;

    bool read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);
    bool write_register(uint8_t reg, uint8_t value);
    bool load_calibration();
};

#endif