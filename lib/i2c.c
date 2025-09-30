/******************************************************************************
 * @file i2c.c
 * @brief I2C driver wrapper function implementations
 *
 * This file implements the BMV080 i2c connection
 *
 * @author Sam Allahverdi
 * @version 1.0
 *
 ******************************************************************************/


#include "i2c_header.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

// Define a log module for this file
LOG_MODULE_REGISTER(i2c_driver, LOG_LEVEL_INF);

int i2c_driver_init(const struct i2c_dt_spec *dev)
{
    if (!device_is_ready(dev->bus)) {
        LOG_ERR("I2C bus %s is not ready!", dev->bus->name);
        return -ENODEV;
    }

    LOG_INF("I2C device on bus %s at address 0x%x is ready.", dev->bus->name, dev->addr);
    return 0;
}

int i2c_driver_write(const struct i2c_dt_spec *dev, uint8_t *data, uint32_t len)
{
    int ret = i2c_write_dt(dev, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to write to I2C device at 0x%x (err %d)", dev->addr, ret);
    }
    return ret;
}

int i2c_driver_read(const struct i2c_dt_spec *dev, uint8_t *data, uint32_t len)
{
    int ret = i2c_read_dt(dev, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to read from I2C device at 0x%x (err %d)", dev->addr, ret);
    }
    return ret;
}

int i2c_driver_write_read(const struct i2c_dt_spec *dev, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    int ret = i2c_write_read_dt(dev, &reg_addr, sizeof(reg_addr), data, len);
    if (ret != 0) {
        LOG_ERR("Failed to write/read I2C device at 0x%x (err %d)", dev->addr, ret);
    }
    return ret;
}