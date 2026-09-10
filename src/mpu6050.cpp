#include "mpu6050.hpp"

MPU6050::MPU6050(uint8_t adress, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin) {
    _adress = adress;
    _i2c_scl_pin = i2c_scl_pin;
    _i2c_sda_pin = i2c_sda_pin;
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(_i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(_i2c_sda_pin, GPIO_FUNC_I2C);
    _config();
}

inline void MPU6050::_write_register(uint8_t reg, uint8_t val) {
  uint8_t data[] = {reg, val};
  i2c_write_blocking(i2c_default, _adress, data, 2, false);
}

inline uint8_t MPU6050::_read_register(uint8_t reg) {
  uint8_t res;
  i2c_write_blocking(i2c_default, _adress, &reg, 1, true);
  i2c_read_blocking(i2c_default, _adress, &res, 6, false);
  return res;
}

void MPU6050::_config() {
    _write_register(MPU6050_ADDRESS, 0x80); // reset MPU6050
    sleep_ms(100);
    _write_register(MPU6050_ADDRESS, 0x00); // wake up MPU6050
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
    _read_register(MPU6050_ACCEL_ADDR);

    accel.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel.z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU6050::_read_gyro_raw() {
    uint8_t buffer[6];
    _read_register(MPU6050_GYRO_ADDR);

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
