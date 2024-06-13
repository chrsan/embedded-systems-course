#pragma once

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "esp_err.h"

struct Mpu6050;

enum Mpu6050AccelConfig {
  MPU6050_ACCEL_FS_2G = 0,
  MPU6050_ACCEL_FS_4G = 1,
  MPU6050_ACCEL_FS_8G = 2,
  MPU6050_ACCEL_FS_16G = 3,
};

float mpu6050_accel_sensitivity(enum Mpu6050AccelConfig accel_config);

enum Mpu6050GyroConfig {
  MPU6050_GYRO_FS_250DPS = 0,
  MPU6050_GYRO_FS_500DPS = 1,
  MPU6050_GYRO_FS_1000DPS = 2,
  MPU6050_GYRO_FS_2000DPS = 3,
};

float mpu6050_gyro_sensitivity(enum Mpu6050GyroConfig gyro_config);

struct Mpu6050Config {
  i2c_port_num_t port;
  gpio_num_t sda_pin;
  gpio_num_t scl_pin;
  enum Mpu6050AccelConfig accel_config;
  enum Mpu6050GyroConfig gyro_config;
};

esp_err_t mpu6050_init(const struct Mpu6050Config* config, uint8_t address,
                       struct Mpu6050** mpu6050);

void mpu6050_deinit(struct Mpu6050* mpu6050);

struct Mpu6050Value {
  int16_t x;
  int16_t y;
  int16_t z;
};

esp_err_t mpu6050_get_accel(const struct Mpu6050* mpu6050,
                            struct Mpu6050Value* value);

esp_err_t mpu6050_get_gyro(const struct Mpu6050* mpu6050,
                           struct Mpu6050Value* value);
