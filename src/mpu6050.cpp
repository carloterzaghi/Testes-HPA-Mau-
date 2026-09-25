#include "mpu6050.hpp"

#include "pico/stdlib.h"

MPU6050::MPU6050(I2CBus& bus, uint8_t address)
    : _bus(bus), _adress(address), _gyro_offset{}, _gyro_raw{} {
    _config();
}

inline void MPU6050::_write_register(uint8_t reg, uint8_t val) {
  uint8_t data[] = {reg, val};
    _bus.write(_adress, data, 2, false);
}

bool MPU6050::_read_register(uint8_t reg, uint8_t* buffer, size_t length) {
    if (_bus.write(_adress, &reg, 1, true) != 1) {
        return false;
    }
    return _bus.read(_adress, buffer, length, false) == static_cast<int>(length);
}

void MPU6050::_config() {
    _write_register(0x6B, 0x80); // reset MPU6050
    sleep_ms(100);
    _write_register(0x6B, 0x00); // wake up MPU6050
    sleep_ms(10);
}

void MPU6050::calibrate() {
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        _read_gyro_raw();
        gyro_x_sum += _gyro_raw.x;
        gyro_y_sum += _gyro_raw.y;
        gyro_z_sum += _gyro_raw.z;
        sleep_ms(1);
    }

    _gyro_offset.x = gyro_x_sum / CALIBRATION_SAMPLES;
    _gyro_offset.y = gyro_y_sum / CALIBRATION_SAMPLES;
    _gyro_offset.z = gyro_z_sum / CALIBRATION_SAMPLES;
}

void MPU6050::_read_accel_raw() {
    uint8_t buffer[6];
    if (!_read_register(MPU6050_ACCEL_ADDR, buffer, sizeof(buffer))) {
        return;
    }

    accel.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel.z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU6050::_read_gyro_raw() {
    uint8_t buffer[6];
    if (!_read_register(MPU6050_GYRO_ADDR, buffer, sizeof(buffer))) {
        return;
    }

    _gyro_raw.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    _gyro_raw.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    _gyro_raw.z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU6050::read_gyro() {
    _read_gyro_raw();
    gyro.x -= _gyro_offset.x;
    gyro.y -= _gyro_offset.y;
    gyro.z -= _gyro_offset.z;
}
