/******************************************************************************
 * @file i2c_header.h
 * @brief I2C header function definition file
 *
 * This file defines functions for the BMV080 i2c connection
 *
 * @author Sam Allahverdi
 * @version 1.0
 *
 ******************************************************************************/

#ifndef I2C_HEADER_H_
#define I2C_HEADER_H_

#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/**
 * @brief Initializes the I2C device specified by the devicetree.
 *
 * This function checks if the I2C bus controller is ready for communication.
 *
 * @param dev Pointer to the i2c_dt_spec structure for the target device.
 * @return 0 on success, or a negative error code on failure.
 */
int i2c_driver_init(const struct i2c_dt_spec *dev);

/**
 * @brief Writes a buffer of data to the I2C device.
 *
 * @param dev Pointer to the i2c_dt_spec structure for the target device.
 * @param data Pointer to the data buffer to write.
 * @param len Number of bytes to write.
 * @return 0 on success, or a negative error code on failure.
 */
int i2c_driver_write(const struct i2c_dt_spec *dev, uint8_t *data, uint32_t len);

/**
 * @brief Reads a buffer of data from the I2C device.
 *
 * @param dev Pointer to the i2c_dt_spec structure for the target device.
 * @param data Pointer to the buffer where read data will be stored.
 * @param len Number of bytes to read.
 * @return 0 on success, or a negative error code on failure.
 */
int i2c_driver_read(const struct i2c_dt_spec *dev, uint8_t *data, uint32_t len);

/**
 * @brief Writes a register address and then reads data from the device.
 *
 * This is a common pattern for reading a specific register from a sensor.
 *
 * @param dev Pointer to the i2c_dt_spec structure for the target device.
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where read data will be stored.
 * @param len Number of bytes to read.
 * @return 0 on success, or a negative error code on failure.
 */
int i2c_driver_write_read(const struct i2c_dt_spec *dev, uint8_t reg_addr, uint8_t *data, uint32_t len);


#endif /* I2C_HEADER_H_ */