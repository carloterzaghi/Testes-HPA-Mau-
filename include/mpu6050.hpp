#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDRESS 0x68
#define MPU6050_I2c_FREQ 400000 // 400 kHz

#define MPU6050_GYRO_ADDR 0x43
#define MPU6050_ACCEL_ADDR 0x3B

#define CALIBRATION_SAMPLES 1000

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} mpu6050_data_t;

class MPU6050 {
    public:
        MPU6050(uint8_t adress, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin);
        void read_accel();
        void read_gyro();
        void calibrate();
        mpu6050_data_t accel;
        mpu6050_data_t gyro;
    private:
        uint8_t _adress = 0x68;
        uint8_t _i2c_scl_pin;
        uint8_t _i2c_sda_pin;
        mpu6050_data_t _gyro_offset;
        mpu6050_data_t _gyro_raw;
        void _config();
        void _read_accel_raw();
        void _read_gyro_raw();
        inline void _write_register(uint8_t reg, uint8_t val);
        inline uint8_t _read_register(uint8_t reg);
};

#endif