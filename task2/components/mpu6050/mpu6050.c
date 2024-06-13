#include "mpu6050.h"

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"

#define TAG "mpu6050"
#define TIMEOUT 1000

float mpu6050_accel_sensitivity(enum Mpu6050AccelConfig accel_config) {
  switch (accel_config) {
    case MPU6050_ACCEL_FS_2G:
      return 16384;
    case MPU6050_ACCEL_FS_4G:
      return 8192;
    case MPU6050_ACCEL_FS_8G:
      return 4096;
    case MPU6050_ACCEL_FS_16G:
      return 2048;
      break;
    default:
      // N.B. Avoid divide by zero.
      return 1;
  }
}

float mpu6050_gyro_sensitivity(enum Mpu6050GyroConfig gyro_config) {
  switch (gyro_config) {
    case MPU6050_GYRO_FS_250DPS:
      return 131;
    case MPU6050_GYRO_FS_500DPS:
      return 65.5;
    case MPU6050_GYRO_FS_1000DPS:
      return 32.8;
    case MPU6050_GYRO_FS_2000DPS:
      return 16.4;
    default:
      // N.B. Avoid divide by zero.
      return 1;
  }
}

struct Mpu6050 {
  enum Mpu6050AccelConfig accel_config;
  enum Mpu6050GyroConfig gyro_config;
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
};

static esp_err_t mpu6050_read(const struct Mpu6050* mpu6050, uint8_t reg,
                              uint8_t* value, uint8_t value_len) {
  return i2c_master_transmit_receive(mpu6050->dev_handle, &reg, 1, value,
                                     value_len, TIMEOUT);
}

static esp_err_t mpu6050_write(const struct Mpu6050* mpu6050, uint8_t reg,
                               uint8_t value) {
  uint8_t buf[2] = {reg, value};
  return i2c_master_transmit(mpu6050->dev_handle, buf, sizeof(buf), TIMEOUT);
}

#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define PWR_MGMT_1 0x6b

esp_err_t mpu6050_init(const struct Mpu6050Config* config, uint8_t address,
                       struct Mpu6050** mpu6050) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG,
                      "config must not be null");
  ESP_RETURN_ON_FALSE(config->port < I2C_NUM_MAX, ESP_FAIL, TAG,
                      "invalid I2C port number: %d", config->port);
  ESP_RETURN_ON_FALSE(mpu6050, ESP_ERR_INVALID_ARG, TAG,
                      "mpu6050 must not be null");

  struct Mpu6050* m = calloc(1, sizeof(struct Mpu6050));
  ESP_RETURN_ON_FALSE(m, ESP_ERR_NO_MEM, TAG, "Mpu6050 alloc error");

  m->gyro_config = config->gyro_config;
  m->accel_config = config->accel_config;

  esp_err_t ret = ESP_OK;

  i2c_master_bus_config_t bus_config = {
      .i2c_port = config->port,
      .sda_io_num = config->sda_pin,
      .scl_io_num = config->scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_GOTO_ON_ERROR(i2c_new_master_bus(&bus_config, &m->bus_handle), err, TAG,
                    "error allocating I2C master bus");

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = 100000,
  };

  ESP_GOTO_ON_ERROR(
      i2c_master_bus_add_device(m->bus_handle, &dev_config, &m->dev_handle),
      err, TAG, "error adding I2C master bus device");

  ESP_GOTO_ON_ERROR(mpu6050_write(m, GYRO_CONFIG, config->gyro_config << 3),
                    err, TAG, "error configuring gyroscope");
  ESP_GOTO_ON_ERROR(mpu6050_write(m, ACCEL_CONFIG, config->accel_config << 3),
                    err, TAG, "error configuring accelerometer");

  uint8_t value = 0;
  ESP_GOTO_ON_ERROR(mpu6050_read(m, PWR_MGMT_1, &value, 1), err, TAG,
                    "could not read PWR_MGMT_1 reg");
  ESP_GOTO_ON_ERROR(mpu6050_write(m, PWR_MGMT_1, value & ~0x64), err, TAG,
                    "could not reset sleep bit in power management 1 register");

  *mpu6050 = m;
  return ESP_OK;
err:
  mpu6050_deinit(m);
  return ret;
}

void mpu6050_deinit(struct Mpu6050* mpu6050) {
  if (mpu6050) {
    if (mpu6050->dev_handle) {
      i2c_master_bus_rm_device(mpu6050->dev_handle);
    }

    if (mpu6050->bus_handle) {
      i2c_del_master_bus(mpu6050->bus_handle);
    }

    free(mpu6050);
  }
}

static esp_err_t mpu6050_read_values(const struct Mpu6050* mpu6050, uint8_t reg,
                                     struct Mpu6050Value* value) {
  uint8_t data[6];
  esp_err_t ret = mpu6050_read(mpu6050, reg, data, sizeof(data));
  if (ret != ESP_OK) {
    return ret;
  }

  value->x = (int16_t)((data[0] << 8) + (data[1]));
  value->y = (int16_t)((data[2] << 8) + (data[3]));
  value->z = (int16_t)((data[4] << 8) + (data[5]));
  return ESP_OK;
}

#define ACCEL_XOUT_H 0x3b

esp_err_t mpu6050_get_accel(const struct Mpu6050* mpu6050,
                            struct Mpu6050Value* value) {
  if (!value) {
    return ESP_OK;
  }

  return mpu6050_read_values(mpu6050, ACCEL_XOUT_H, value);
}

#define GYRO_XOUT_H 0x43

esp_err_t mpu6050_get_gyro(const struct Mpu6050* mpu6050,
                           struct Mpu6050Value* value) {
  if (!value) {
    return ESP_OK;
  }

  return mpu6050_read_values(mpu6050, GYRO_XOUT_H, value);
}
